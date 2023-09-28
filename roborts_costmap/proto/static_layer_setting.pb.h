// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: static_layer_setting.proto

#ifndef PROTOBUF_static_5flayer_5fsetting_2eproto__INCLUDED
#define PROTOBUF_static_5flayer_5fsetting_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2006000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace roborts_costmap {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_static_5flayer_5fsetting_2eproto();
void protobuf_AssignDesc_static_5flayer_5fsetting_2eproto();
void protobuf_ShutdownFile_static_5flayer_5fsetting_2eproto();

class ParaStaticLayer;

// ===================================================================

class ParaStaticLayer : public ::google::protobuf::Message {
 public:
  ParaStaticLayer();
  virtual ~ParaStaticLayer();

  ParaStaticLayer(const ParaStaticLayer& from);

  inline ParaStaticLayer& operator=(const ParaStaticLayer& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const ParaStaticLayer& default_instance();

  void Swap(ParaStaticLayer* other);

  // implements Message ----------------------------------------------

  ParaStaticLayer* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const ParaStaticLayer& from);
  void MergeFrom(const ParaStaticLayer& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required bool first_map_only = 1;
  inline bool has_first_map_only() const;
  inline void clear_first_map_only();
  static const int kFirstMapOnlyFieldNumber = 1;
  inline bool first_map_only() const;
  inline void set_first_map_only(bool value);

  // required bool subscribe_to_updates = 2;
  inline bool has_subscribe_to_updates() const;
  inline void clear_subscribe_to_updates();
  static const int kSubscribeToUpdatesFieldNumber = 2;
  inline bool subscribe_to_updates() const;
  inline void set_subscribe_to_updates(bool value);

  // required bool track_unknown_space = 3;
  inline bool has_track_unknown_space() const;
  inline void clear_track_unknown_space();
  static const int kTrackUnknownSpaceFieldNumber = 3;
  inline bool track_unknown_space() const;
  inline void set_track_unknown_space(bool value);

  // required bool use_maximum = 4;
  inline bool has_use_maximum() const;
  inline void clear_use_maximum();
  static const int kUseMaximumFieldNumber = 4;
  inline bool use_maximum() const;
  inline void set_use_maximum(bool value);

  // required int32 unknown_cost_value = 5;
  inline bool has_unknown_cost_value() const;
  inline void clear_unknown_cost_value();
  static const int kUnknownCostValueFieldNumber = 5;
  inline ::google::protobuf::int32 unknown_cost_value() const;
  inline void set_unknown_cost_value(::google::protobuf::int32 value);

  // required bool trinary_map = 6;
  inline bool has_trinary_map() const;
  inline void clear_trinary_map();
  static const int kTrinaryMapFieldNumber = 6;
  inline bool trinary_map() const;
  inline void set_trinary_map(bool value);

  // required int32 lethal_threshold = 7;
  inline bool has_lethal_threshold() const;
  inline void clear_lethal_threshold();
  static const int kLethalThresholdFieldNumber = 7;
  inline ::google::protobuf::int32 lethal_threshold() const;
  inline void set_lethal_threshold(::google::protobuf::int32 value);

  // required string topic_name = 8;
  inline bool has_topic_name() const;
  inline void clear_topic_name();
  static const int kTopicNameFieldNumber = 8;
  inline const ::std::string& topic_name() const;
  inline void set_topic_name(const ::std::string& value);
  inline void set_topic_name(const char* value);
  inline void set_topic_name(const char* value, size_t size);
  inline ::std::string* mutable_topic_name();
  inline ::std::string* release_topic_name();
  inline void set_allocated_topic_name(::std::string* topic_name);

  // required bool is_raw_rosmessage = 9;
  inline bool has_is_raw_rosmessage() const;
  inline void clear_is_raw_rosmessage();
  static const int kIsRawRosmessageFieldNumber = 9;
  inline bool is_raw_rosmessage() const;
  inline void set_is_raw_rosmessage(bool value);

  // required bool is_debug = 10;
  inline bool has_is_debug() const;
  inline void clear_is_debug();
  static const int kIsDebugFieldNumber = 10;
  inline bool is_debug() const;
  inline void set_is_debug(bool value);

  // @@protoc_insertion_point(class_scope:roborts_costmap.ParaStaticLayer)
 private:
  inline void set_has_first_map_only();
  inline void clear_has_first_map_only();
  inline void set_has_subscribe_to_updates();
  inline void clear_has_subscribe_to_updates();
  inline void set_has_track_unknown_space();
  inline void clear_has_track_unknown_space();
  inline void set_has_use_maximum();
  inline void clear_has_use_maximum();
  inline void set_has_unknown_cost_value();
  inline void clear_has_unknown_cost_value();
  inline void set_has_trinary_map();
  inline void clear_has_trinary_map();
  inline void set_has_lethal_threshold();
  inline void clear_has_lethal_threshold();
  inline void set_has_topic_name();
  inline void clear_has_topic_name();
  inline void set_has_is_raw_rosmessage();
  inline void clear_has_is_raw_rosmessage();
  inline void set_has_is_debug();
  inline void clear_has_is_debug();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  bool first_map_only_;
  bool subscribe_to_updates_;
  bool track_unknown_space_;
  bool use_maximum_;
  ::google::protobuf::int32 unknown_cost_value_;
  ::std::string* topic_name_;
  ::google::protobuf::int32 lethal_threshold_;
  bool trinary_map_;
  bool is_raw_rosmessage_;
  bool is_debug_;
  friend void  protobuf_AddDesc_static_5flayer_5fsetting_2eproto();
  friend void protobuf_AssignDesc_static_5flayer_5fsetting_2eproto();
  friend void protobuf_ShutdownFile_static_5flayer_5fsetting_2eproto();

  void InitAsDefaultInstance();
  static ParaStaticLayer* default_instance_;
};
// ===================================================================


// ===================================================================

// ParaStaticLayer

// required bool first_map_only = 1;
inline bool ParaStaticLayer::has_first_map_only() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void ParaStaticLayer::set_has_first_map_only() {
  _has_bits_[0] |= 0x00000001u;
}
inline void ParaStaticLayer::clear_has_first_map_only() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void ParaStaticLayer::clear_first_map_only() {
  first_map_only_ = false;
  clear_has_first_map_only();
}
inline bool ParaStaticLayer::first_map_only() const {
  // @@protoc_insertion_point(field_get:roborts_costmap.ParaStaticLayer.first_map_only)
  return first_map_only_;
}
inline void ParaStaticLayer::set_first_map_only(bool value) {
  set_has_first_map_only();
  first_map_only_ = value;
  // @@protoc_insertion_point(field_set:roborts_costmap.ParaStaticLayer.first_map_only)
}

// required bool subscribe_to_updates = 2;
inline bool ParaStaticLayer::has_subscribe_to_updates() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void ParaStaticLayer::set_has_subscribe_to_updates() {
  _has_bits_[0] |= 0x00000002u;
}
inline void ParaStaticLayer::clear_has_subscribe_to_updates() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void ParaStaticLayer::clear_subscribe_to_updates() {
  subscribe_to_updates_ = false;
  clear_has_subscribe_to_updates();
}
inline bool ParaStaticLayer::subscribe_to_updates() const {
  // @@protoc_insertion_point(field_get:roborts_costmap.ParaStaticLayer.subscribe_to_updates)
  return subscribe_to_updates_;
}
inline void ParaStaticLayer::set_subscribe_to_updates(bool value) {
  set_has_subscribe_to_updates();
  subscribe_to_updates_ = value;
  // @@protoc_insertion_point(field_set:roborts_costmap.ParaStaticLayer.subscribe_to_updates)
}

// required bool track_unknown_space = 3;
inline bool ParaStaticLayer::has_track_unknown_space() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void ParaStaticLayer::set_has_track_unknown_space() {
  _has_bits_[0] |= 0x00000004u;
}
inline void ParaStaticLayer::clear_has_track_unknown_space() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void ParaStaticLayer::clear_track_unknown_space() {
  track_unknown_space_ = false;
  clear_has_track_unknown_space();
}
inline bool ParaStaticLayer::track_unknown_space() const {
  // @@protoc_insertion_point(field_get:roborts_costmap.ParaStaticLayer.track_unknown_space)
  return track_unknown_space_;
}
inline void ParaStaticLayer::set_track_unknown_space(bool value) {
  set_has_track_unknown_space();
  track_unknown_space_ = value;
  // @@protoc_insertion_point(field_set:roborts_costmap.ParaStaticLayer.track_unknown_space)
}

// required bool use_maximum = 4;
inline bool ParaStaticLayer::has_use_maximum() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void ParaStaticLayer::set_has_use_maximum() {
  _has_bits_[0] |= 0x00000008u;
}
inline void ParaStaticLayer::clear_has_use_maximum() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void ParaStaticLayer::clear_use_maximum() {
  use_maximum_ = false;
  clear_has_use_maximum();
}
inline bool ParaStaticLayer::use_maximum() const {
  // @@protoc_insertion_point(field_get:roborts_costmap.ParaStaticLayer.use_maximum)
  return use_maximum_;
}
inline void ParaStaticLayer::set_use_maximum(bool value) {
  set_has_use_maximum();
  use_maximum_ = value;
  // @@protoc_insertion_point(field_set:roborts_costmap.ParaStaticLayer.use_maximum)
}

// required int32 unknown_cost_value = 5;
inline bool ParaStaticLayer::has_unknown_cost_value() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void ParaStaticLayer::set_has_unknown_cost_value() {
  _has_bits_[0] |= 0x00000010u;
}
inline void ParaStaticLayer::clear_has_unknown_cost_value() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void ParaStaticLayer::clear_unknown_cost_value() {
  unknown_cost_value_ = 0;
  clear_has_unknown_cost_value();
}
inline ::google::protobuf::int32 ParaStaticLayer::unknown_cost_value() const {
  // @@protoc_insertion_point(field_get:roborts_costmap.ParaStaticLayer.unknown_cost_value)
  return unknown_cost_value_;
}
inline void ParaStaticLayer::set_unknown_cost_value(::google::protobuf::int32 value) {
  set_has_unknown_cost_value();
  unknown_cost_value_ = value;
  // @@protoc_insertion_point(field_set:roborts_costmap.ParaStaticLayer.unknown_cost_value)
}

// required bool trinary_map = 6;
inline bool ParaStaticLayer::has_trinary_map() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void ParaStaticLayer::set_has_trinary_map() {
  _has_bits_[0] |= 0x00000020u;
}
inline void ParaStaticLayer::clear_has_trinary_map() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void ParaStaticLayer::clear_trinary_map() {
  trinary_map_ = false;
  clear_has_trinary_map();
}
inline bool ParaStaticLayer::trinary_map() const {
  // @@protoc_insertion_point(field_get:roborts_costmap.ParaStaticLayer.trinary_map)
  return trinary_map_;
}
inline void ParaStaticLayer::set_trinary_map(bool value) {
  set_has_trinary_map();
  trinary_map_ = value;
  // @@protoc_insertion_point(field_set:roborts_costmap.ParaStaticLayer.trinary_map)
}

// required int32 lethal_threshold = 7;
inline bool ParaStaticLayer::has_lethal_threshold() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void ParaStaticLayer::set_has_lethal_threshold() {
  _has_bits_[0] |= 0x00000040u;
}
inline void ParaStaticLayer::clear_has_lethal_threshold() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void ParaStaticLayer::clear_lethal_threshold() {
  lethal_threshold_ = 0;
  clear_has_lethal_threshold();
}
inline ::google::protobuf::int32 ParaStaticLayer::lethal_threshold() const {
  // @@protoc_insertion_point(field_get:roborts_costmap.ParaStaticLayer.lethal_threshold)
  return lethal_threshold_;
}
inline void ParaStaticLayer::set_lethal_threshold(::google::protobuf::int32 value) {
  set_has_lethal_threshold();
  lethal_threshold_ = value;
  // @@protoc_insertion_point(field_set:roborts_costmap.ParaStaticLayer.lethal_threshold)
}

// required string topic_name = 8;
inline bool ParaStaticLayer::has_topic_name() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void ParaStaticLayer::set_has_topic_name() {
  _has_bits_[0] |= 0x00000080u;
}
inline void ParaStaticLayer::clear_has_topic_name() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void ParaStaticLayer::clear_topic_name() {
  if (topic_name_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    topic_name_->clear();
  }
  clear_has_topic_name();
}
inline const ::std::string& ParaStaticLayer::topic_name() const {
  // @@protoc_insertion_point(field_get:roborts_costmap.ParaStaticLayer.topic_name)
  return *topic_name_;
}
inline void ParaStaticLayer::set_topic_name(const ::std::string& value) {
  set_has_topic_name();
  if (topic_name_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    topic_name_ = new ::std::string;
  }
  topic_name_->assign(value);
  // @@protoc_insertion_point(field_set:roborts_costmap.ParaStaticLayer.topic_name)
}
inline void ParaStaticLayer::set_topic_name(const char* value) {
  set_has_topic_name();
  if (topic_name_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    topic_name_ = new ::std::string;
  }
  topic_name_->assign(value);
  // @@protoc_insertion_point(field_set_char:roborts_costmap.ParaStaticLayer.topic_name)
}
inline void ParaStaticLayer::set_topic_name(const char* value, size_t size) {
  set_has_topic_name();
  if (topic_name_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    topic_name_ = new ::std::string;
  }
  topic_name_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:roborts_costmap.ParaStaticLayer.topic_name)
}
inline ::std::string* ParaStaticLayer::mutable_topic_name() {
  set_has_topic_name();
  if (topic_name_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    topic_name_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:roborts_costmap.ParaStaticLayer.topic_name)
  return topic_name_;
}
inline ::std::string* ParaStaticLayer::release_topic_name() {
  clear_has_topic_name();
  if (topic_name_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = topic_name_;
    topic_name_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void ParaStaticLayer::set_allocated_topic_name(::std::string* topic_name) {
  if (topic_name_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete topic_name_;
  }
  if (topic_name) {
    set_has_topic_name();
    topic_name_ = topic_name;
  } else {
    clear_has_topic_name();
    topic_name_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:roborts_costmap.ParaStaticLayer.topic_name)
}

// required bool is_raw_rosmessage = 9;
inline bool ParaStaticLayer::has_is_raw_rosmessage() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void ParaStaticLayer::set_has_is_raw_rosmessage() {
  _has_bits_[0] |= 0x00000100u;
}
inline void ParaStaticLayer::clear_has_is_raw_rosmessage() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void ParaStaticLayer::clear_is_raw_rosmessage() {
  is_raw_rosmessage_ = false;
  clear_has_is_raw_rosmessage();
}
inline bool ParaStaticLayer::is_raw_rosmessage() const {
  // @@protoc_insertion_point(field_get:roborts_costmap.ParaStaticLayer.is_raw_rosmessage)
  return is_raw_rosmessage_;
}
inline void ParaStaticLayer::set_is_raw_rosmessage(bool value) {
  set_has_is_raw_rosmessage();
  is_raw_rosmessage_ = value;
  // @@protoc_insertion_point(field_set:roborts_costmap.ParaStaticLayer.is_raw_rosmessage)
}

// required bool is_debug = 10;
inline bool ParaStaticLayer::has_is_debug() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void ParaStaticLayer::set_has_is_debug() {
  _has_bits_[0] |= 0x00000200u;
}
inline void ParaStaticLayer::clear_has_is_debug() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void ParaStaticLayer::clear_is_debug() {
  is_debug_ = false;
  clear_has_is_debug();
}
inline bool ParaStaticLayer::is_debug() const {
  // @@protoc_insertion_point(field_get:roborts_costmap.ParaStaticLayer.is_debug)
  return is_debug_;
}
inline void ParaStaticLayer::set_is_debug(bool value) {
  set_has_is_debug();
  is_debug_ = value;
  // @@protoc_insertion_point(field_set:roborts_costmap.ParaStaticLayer.is_debug)
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace roborts_costmap

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_static_5flayer_5fsetting_2eproto__INCLUDED