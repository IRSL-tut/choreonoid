/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiValueSeq.h"
#include "PlainSeqFileLoader.h"
#include "ValueTree.h"
#include "YAMLWriter.h"
#include "GeneralSeqReader.h"
#include <fstream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using boost::format;


MultiValueSeq::MultiValueSeq()
    : BaseSeqType("MultiValueSeq")
{

}


MultiValueSeq::MultiValueSeq(int numFrames, int numParts)
    : BaseSeqType("MultiValueSeq", numFrames, numParts)
{

}


MultiValueSeq::MultiValueSeq(const MultiValueSeq& org)
    : BaseSeqType(org)
{

}


AbstractSeqPtr MultiValueSeq::cloneSeq() const
{
    return std::make_shared<MultiValueSeq>(*this);
}


MultiValueSeq::~MultiValueSeq()
{

}


bool MultiValueSeq::doReadSeq(const Mapping& archive, std::ostream& os)
{
    GeneralSeqReader reader(os);
    return reader.read(&archive, this, [](const ValueNode& node, double& v){ v = node.toDouble(); });
}
    

bool MultiValueSeq::doWriteSeq(YAMLWriter& writer)
{
    if(!BaseSeqType::doWriteSeq(writer)){
        return false;
    }

    writer.putKey("frames");
    writer.startListing();
    const int n = numFrames();
    const int m = numParts();
    for(int i=0; i < n; ++i){
        writer.startFlowStyleListing();
        Frame v = frame(i);
        for(int j=0; j < m; ++j){
            writer.putScalar(v[j]);
        }
        writer.endListing();
    }
    writer.endListing();
    return true;
}


bool MultiValueSeq::loadPlainFormat(const std::string& filename, std::ostream& os)
{
    PlainSeqFileLoader loader;

    if(!loader.load(filename, os)){
        return false;
    }

    setDimension(loader.numFrames(), loader.numParts());
    setFrameRate(1.0 / loader.timeStep());

    int i = 0;
    for(PlainSeqFileLoader::iterator it = loader.begin(); it != loader.end(); ++it){
        copy((it->begin() + 1), it->end(), frame(i++).begin());
    }

    return true;
}


bool MultiValueSeq::saveAsPlainFormat(const std::string& filename, std::ostream& os)
{
    ofstream file(filename.c_str());
    file.setf(ios::fixed);

    if(!file){
        os << format(_("\"%1%\" cannot be opened.")) % filename << endl;
        return false;
    }

    const int n = numFrames();
    const int m = numParts();
    const double r = frameRate();

    for(int i=0; i < n; ++i){
        file << (i / r);
        Frame v = frame(i);
        for(int j=0; j < m; ++j){
            file << " " << v[j];
        }
        file << "\n";
    }
    
    return true;
}
