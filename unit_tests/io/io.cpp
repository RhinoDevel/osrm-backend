#include "extractor/extractor_options.hpp"
#include "extractor/extractor.hpp"

#include "contractor/contractor_options.hpp"
#include "contractor/processing_chain.hpp"

#include "util/typedefs.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/test/test_case_template.hpp>

#include <string>

const static std::string IO_TMP_FILE = "test_io.tmp";

class ExtractorAccess : protected Extractor
{
  public:
    ExtractorAccess() : Extractor(ExtractorConfig())
    {
        super::config.oneway_flags_file_name = IO_TMP_FILE;
    }

    void WriteFlags(const std::vector<bool> &flags) { super::WriteOneWayFlags(flags); }
};

class PrepareAccess : protected Prepare
{
  public:
    PrepareAccess() : Prepare(ContractorConfig())
    {
        super::config.one_way_flags_path = IO_TMP_FILE;
    }

    void ReadFlags(std::vector<bool> &flags) { super::ReadOneWayFlags(flags); }
};

BOOST_AUTO_TEST_SUITE(osrm_io)

BOOST_AUTO_TEST_CASE(io_flags)
{
    std::vector<bool> flags_in, flags_out;
    flags_in.resize(53);
    for (std::size_t i = 0; i < flags_in.size(); ++i)
        flags_in[i] = ((i % 2) == 1);

    ExtractorAccess extractor_access;
    extractor_access.WriteFlags(flags_in);

    PrepareAccess prepare_access;
    prepare_access.ReadFlags(flags_out);

    BOOST_REQUIRE_EQUAL(flags_in.size(), flags_out.size());
    BOOST_CHECK_EQUAL_COLLECTIONS(flags_out.begin(), flags_out.end(), flags_in.begin(),
                                  flags_in.end());
}
