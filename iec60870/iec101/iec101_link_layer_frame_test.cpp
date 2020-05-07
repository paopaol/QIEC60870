#include "iec101_link_layer_frame.h"

#include <gtest/gtest.h>

using namespace testing;
using namespace QIEC60870::p101;

TEST(LinkLayerFrameCodec, decode_works_well) {
  struct TestCase {
    std::vector<uint8_t> data;
    FrameParseErr error;
    std::string name;
  };
  std::vector<TestCase> cases = {
      {{0x10, 0x5a, 0x01, 0x5b, 0x16}, FrameParseErr::kNoError, "case0"},
      {{0x10, 0x5a, 0x01, 0x5c, 0x16}, FrameParseErr::kCheckError, "case1"},
      {{0x40, 0x5a, 0x01, 0x5b, 0x16}, FrameParseErr::kBadFormat, "cased2"},
      {{0x10, 0x5a, 0x01, 0x5b, 0x26}, FrameParseErr::kBadFormat, "case3"},
      {{0x10, 0x5a}, FrameParseErr::kNeedMoreData, "case4"},
  };

  for (const auto &test : cases) {
    LinkLayerFrame frame;
    LinkLayerFrameCodec codec;
    codec.decode(test.data);

    EXPECT_EQ(codec.error(), test.error) << test.name;

    if (codec.error() == FrameParseErr::kNoError) {
      frame = codec.toLinkLayerFrame();
      EXPECT_EQ(frame.c().raw(), 0x5a);
    }
  }
}
