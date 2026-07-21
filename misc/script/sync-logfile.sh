#!/bin/bash

# Synchronize a local file to a remote host in (near) real time.
# This is mainly used for synchronizing a world log file being recorded by
# a simulation so that another Choreonoid on the remote host can play it
# back with the live playback mode of WorldLogFileItem.

# Function to display usage information
usage() {
    echo "Usage: $0 SOURCE_FILE REMOTE_HOST [REMOTE_FILE]"
    echo "  SOURCE_FILE: Path to the local source file"
    echo "  REMOTE_HOST: Remote host in the format user@hostname"
    echo "  REMOTE_FILE: (Optional) Path to the remote destination file. If omitted, same as SOURCE_FILE"
    exit 1
}

# Check command-line arguments
if [ $# -lt 2 ]; then
    usage
fi

# Set parameters
SOURCE_FILE="$1"
REMOTE_HOST="$2"
REMOTE_FILE="${3:-$SOURCE_FILE}"

# Quote the remote file path so that it is treated as a single literal
# argument by the remote shell
REMOTE_FILE_QUOTED=$(printf '%q' "$REMOTE_FILE")

# Interval of checking the source file update in seconds. The default read
# interval of the live playback mode of WorldLogFileItem is 10 ms, so a
# shorter interval than this is not useful.
POLLING_INTERVAL=0.01

# Use a control socket dedicated to this process so that multiple instances
# of this script (e.g. for synchronizing to multiple remote hosts) do not
# conflict with each other
SOCKET_DIR=$(mktemp -d "${TMPDIR:-/tmp}/sync-logfile.XXXXXX")
SSH_SOCKET="$SOCKET_DIR/ssh_socket"

cleanup() {
    trap - EXIT INT TERM
    echo "Cleaning up and closing SSH connection..."
    if [ -e "$SSH_SOCKET" ]; then
        ssh -S "$SSH_SOCKET" -O exit "$REMOTE_HOST" 2>/dev/null
    fi
    rm -rf "$SOCKET_DIR"
    exit 0
}

# Execute cleanup on script termination
trap cleanup EXIT INT TERM

# Establish the SSH master connection and keep it in the background
if ! ssh -N -f -M -S "$SSH_SOCKET" "$REMOTE_HOST"; then
    echo "Error: Cannot establish the SSH connection to $REMOTE_HOST."
    exit 1
fi

# Wait for the source file to appear if it does not exist yet. This allows
# the synchronization to be started before the simulation (and hence the
# log file) is created.
if [ ! -e "$SOURCE_FILE" ]; then
    echo "Waiting for \"$SOURCE_FILE\" to be created..."
    while [ ! -e "$SOURCE_FILE" ]; do
        sleep "$POLLING_INTERVAL"
    done
    echo "\"$SOURCE_FILE\" has been created."
fi

# Copy the whole source file to the remote file. Used for the initial sync
# and for the recovery from a file shrink or a transfer error.
# The synced file size is stored in the last_size variable.
full_sync() {
    local size
    size=$(stat -c %s "$SOURCE_FILE")
    if scp -q -o "ControlPath=$SSH_SOCKET" "$SOURCE_FILE" "$REMOTE_HOST:$REMOTE_FILE_QUOTED"; then
        last_size=$size
        return 0
    fi
    return 1
}

# Append the specified byte range of the source file to the remote file.
# The transferred length is stored in the synced_bytes variable.
sync_range() {
    local start=$1
    local end=$2
    local length=$((end - start))
    synced_bytes=0

    if [ "$length" -gt 0 ]; then
        tail -c "+$((start + 1))" "$SOURCE_FILE" | head -c "$length" |
            ssh -S "$SSH_SOCKET" "$REMOTE_HOST" "cat >> $REMOTE_FILE_QUOTED"
        local status=("${PIPESTATUS[@]}")
        if [ "${status[0]}" -ne 0 ] || [ "${status[2]}" -ne 0 ]; then
            return 1
        fi
        synced_bytes=$length
    fi
    return 0
}

# Perform the initial sync
echo "Performing initial sync..."
if full_sync; then
    echo "Initial sync completed successfully."
else
    echo "Error during initial sync. Exiting."
    exit 1
fi

total_synced_bytes=0
sync_count=0
last_output_seconds=$SECONDS

while true; do
    # Get current file size
    current_size=$(stat -c %s "$SOURCE_FILE" 2>/dev/null)

    if [ -z "$current_size" ]; then
        : # The source file is temporarily missing; just wait for it

    elif [ "$current_size" -lt "$last_size" ]; then
        # The file has shrunk (probably a new recording has started);
        # replace the entire remote file
        if full_sync; then
            echo "File size decreased. Replaced entire file. New size: $last_size bytes"
            total_synced_bytes=$((total_synced_bytes + last_size))
            sync_count=$((sync_count + 1))
        else
            echo "Error in replacing the remote file. Retrying..."
        fi

    elif [ "$current_size" -gt "$last_size" ]; then
        # Send only the newly added data
        if sync_range "$last_size" "$current_size"; then
            total_synced_bytes=$((total_synced_bytes + synced_bytes))
            sync_count=$((sync_count + 1))
            last_size=$current_size
        else
            # The remote file may be partially updated; recover it by
            # replacing the entire file
            echo "Error in appending data. Replacing the entire remote file..."
            full_sync
        fi
    fi

    # Output statistics approximately once per second, only if there were syncs
    if [ $((SECONDS - last_output_seconds)) -ge 1 ]; then
        if [ "$sync_count" -gt 0 ]; then
            echo "Sync count: $sync_count, Total bytes synced: $total_synced_bytes"
        fi
        last_output_seconds=$SECONDS
        sync_count=0
        total_synced_bytes=0
    fi

    sleep "$POLLING_INTERVAL"
done
