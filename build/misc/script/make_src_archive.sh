#!/bin/sh

mkdir choreonoid-2.0.0
git archive v2.0.0 | tar -x -C choreonoid-2.0.0
rm choreonoid-2.0.0/.??*
zip -q choreonoid-2.0.0.zip -r choreonoid-2.0.0
rm -fr choreonoid-2.0.0
