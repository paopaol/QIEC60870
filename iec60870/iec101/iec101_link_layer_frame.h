#ifndef IEC_APDU_H
#define IEC_APDU_H

#include <vector>

namespace QIEC60870 {
namespace p101 {
enum class FrameParseErr {
  kNoError = 0,
  kNeedMoreData = 1,
  kBadFormat = 2,
  kCheckError = 3
};

enum class PRM { kFromStartupStation = 1, kFromSlaveStation = 0 };
enum class DIR { kFromMasterStation = 0, kFromSlaveStation = 1 };
enum class FCB { k0, k1 };
enum class FCV { kFCBValid = 1, kFCBInvalid = 0 };
enum class ACD { kLevel1DataWatingAccess = 1, kLevel1NoDataWatingAccess = 0 };
enum class DFC { kSlaveCannotRecv = 1, kSlaveCanRecv = 0 };

enum class StartupFunction {
  kResetRemoteLink = 0,
  kSendLinkStatus = 2,
  kSendUserData = 3,
  kSendNoanswerUserData = 4,
  kAccessRequest = 8,
  kRequestLinkStatus = 9,
  kRequestLevel1UserData = 10,
  kRequestLevel2UserData = 11,
};
enum class SlaveFunction {
  kConfirmedRecognized = 0,
  kConfirmedRejected = 1,
  kResponseUserData = 8,
  kResponseNotFoundUserData = 9,
  kResponseLinkStatus = 11,
};

const int kInvalidA = 0x00;
const int kBroadcastA = 0xffff;

/**
 * @brief Can describe both fixed frames and variable-length frames
 * if asdu_ is empty, it's a fixed frame, otherwise it's a variable frame
 */
class LinkLayerFrame {
public:
  LinkLayerFrame() = default;
  ~LinkLayerFrame() = default;

  LinkLayerFrame(uint8_t c, uint16_t a,
                 const std::vector<uint8_t> &asdu = std::vector<uint8_t>())
      : C(c), A(a), asdu_(asdu) {}

  /**
   * @brief PRM
   *
   * @return
   */
  bool isFromStartupStation() const { return ((C & 0x40) >> 6); }
  /**
   * @brief DIR
   *
   * @return
   */
  bool isFromMasterStation() const { return !((C & 0x80) >> 7); }
  /**
   * @brief FCB
   *
   * @return
   */
  bool fcb() const { return ((C & 0x20) >> 5); }
  /**
   * @brief ACD
   *
   * @return
   */
  bool hasLevel1DataWatingAccess() const { return ((C & 20) >> 5); }
  /**
   * @brief FCV
   *
   * @return
   */
  bool isValidFCB() const { return ((C & 0x10) >> 4); }
  /**
   * @brief DFC
   *
   * @return
   */
  bool isSlaveCannotRecv() const { return ((C & 0x10) >> 4); }
  /**
   * @brief FC
   *
   * @return
   */
  int functionCode() const { return C & 0x0f; }

  void setPRM(PRM prm) {
    C &= 0xbf;
    C |= (prm == PRM::kFromStartupStation ? 0x40 : 0x00);
  }

  void setDIR(DIR dir) {}
  void setFCB(FCB fcb) {}
  void setACD(ACD acd) {}
  void setFCV(FCV fcv) {}
  void setDFC(DFC dfc) {}
  void setFC(int fc) {}

  uint8_t ctrlDomain() const { return C; }

  /**
   * @brief if asdu is empty, that is, it's a fixed frame
   *
   * @return
   */
  bool hasAsdu() { return !asdu_.empty(); }

  std::vector<uint8_t> asdu() const { return asdu_; }
  int address() const { return A; }

  std::vector<uint8_t> encode() {
    bool isFixedFrame = asdu_.empty();
    std::vector<uint8_t> raw;
    if (isFixedFrame) {
      raw.push_back(0x10);
      raw.push_back(C);
      raw.push_back(static_cast<uint8_t>(A));
      raw.push_back(C + A); /// cs
      raw.push_back(0x16);
    } else {
      raw.push_back(0x68);
      uint8_t len = 2 + asdu_.size();
      raw.push_back(len);
      raw.push_back(len);
      raw.push_back(0x68);
      raw.push_back(C);
      raw.push_back(static_cast<uint8_t>(A));
      raw.insert(raw.end(), asdu_.begin(), asdu_.end());
      uint8_t cs = C + A;
      for (const auto &ch : asdu_) {
        cs += ch;
      }
      raw.push_back(cs);
      raw.push_back(0x16);
    }
    return raw;
  }

private:
  uint8_t C = 0x00;
  uint16_t A = kInvalidA;
  std::vector<uint8_t> asdu_;
};

class LinkLayerFrameCodec {
  enum State {
    kStart,
    kSecond68,
    kCtrlDomain,
    kAddressOffset0,
    kLengthOffset0,
    kLengthOffset1,
    kAsdu,
    kCs,
    kEnd,
    kDone
  };

public:
  /**
   * @brief decode decode raw data,
   * parse ctrlDomain,address,asdu
   * after decode, if the error()is FrameParseErr::kNoError,
   * then can call toLinkLayerFrame()
   *
   * @param data
   */
  void decode(const std::vector<uint8_t> &data) {
    for (const auto &ch : data) {
      data_.push_back(ch);
      switch (state_) {
      case kStart: {
        if (ch == 0x10) {
          isFixedFrame_ = true;
          state_ = kCtrlDomain;
        } else if (ch == 0x68) {
          isFixedFrame_ = false;
          state_ = kLengthOffset0;
        } else {
          err_ = FrameParseErr::kBadFormat;
          state_ = kDone;
        }
      } break;
      case kCtrlDomain: {
        ctrlDomain_ = ch;
        state_ = kAddressOffset0;
      } break;
      case kLengthOffset0: {
        length_[0] = ch;
        state_ = kLengthOffset1;
      } break;
      case kLengthOffset1: {
        length_[1] = ch;
        state_ = kSecond68;
      } break;
      case kSecond68: {
        if (ch != 0x68) {
          err_ = FrameParseErr::kBadFormat;
          state_ = kDone;
        } else {
          state_ = kCtrlDomain;
        }
      } break;
      case kAddressOffset0: {
        address_ = ch;
        state_ = isFixedFrame_ ? kCs : kAsdu;
      } break;
      case kAsdu: {
        asdu_.push_back(ch);
        if (length_[0] == asdu_.size() + 2) {
          state_ = kCs;
        }
      } break;
      case kCs: {
        cs_ = ch;
        state_ = kEnd;
      } break;
      case kEnd: {
        err_ = ch == 0x16 ? FrameParseErr::kNoError : FrameParseErr::kBadFormat;
        state_ = kDone;
      } break;
      case kDone: {

      } break;
      }

      if (state_ == kDone) {
        break;
      }
    }
    if (err_ == FrameParseErr::kNoError) {
      uint8_t cs = calculateCs_();
      if (cs != cs_) {
        err_ = FrameParseErr::kCheckError;
      }
    }
  }

  /**
   * @brief if isDone( is true, then can call
   * error() to check decode is failed or successed,
   * if decode is successed, the can call toLinkLayerFrame(
   *
   * @return
   */
  FrameParseErr error() { return err_; }
  LinkLayerFrame toLinkLayerFrame() const {
    return LinkLayerFrame(ctrlDomain_, 3, asdu_);
  }

private:
  uint8_t calculateCs_() {
    uint8_t cs = 0;
    cs += ctrlDomain_;
    cs += address_;
    for (const auto &ch : asdu_) {
      cs += ch;
    }
    return cs;
  }

  uint8_t ctrlDomain_;
  uint8_t address_;
  uint8_t length_[2];
  uint8_t cs_;
  std::vector<uint8_t> asdu_;

  /// internal
  FrameParseErr err_ = FrameParseErr::kNeedMoreData;
  bool isFixedFrame_ = false;
  std::vector<uint8_t> data_;
  State state_ = kStart;
};

} // namespace p101
} // namespace QIEC60870

#endif
