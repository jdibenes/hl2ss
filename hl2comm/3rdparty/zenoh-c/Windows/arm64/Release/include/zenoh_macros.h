#pragma once

#ifndef __cplusplus

// clang-format off
#define z_loan(x) \
    _Generic((x), z_owned_session_t : z_session_loan,                 \
                  z_owned_keyexpr_t : z_keyexpr_loan,                 \
                  z_owned_config_t : z_config_loan,                   \
                  z_owned_publisher_t : z_publisher_loan,             \
                  z_owned_subscriber_t : z_subscriber_loan,           \
                  z_owned_pull_subscriber_t : z_pull_subscriber_loan, \
                  z_owned_encoding_t : z_encoding_loan,               \
                  z_owned_hello_t : z_hello_loan,                     \
                  z_owned_str_t : z_str_loan                          \
            )(&x)

#define z_drop(x) \
    _Generic((x), z_owned_session_t * : z_close,                                    \
                  z_owned_publisher_t * : z_undeclare_publisher,                    \
                  z_owned_keyexpr_t * : z_keyexpr_drop,                             \
                  z_owned_config_t * : z_config_drop,                               \
                  z_owned_scouting_config_t * : z_scouting_config_drop,             \
                  z_owned_pull_subscriber_t * : z_undeclare_pull_subscriber,        \
                  z_owned_subscriber_t * : z_undeclare_subscriber,                  \
                  z_owned_queryable_t * : z_undeclare_queryable,                    \
                  z_owned_encoding_t * : z_encoding_drop,                           \
                  z_owned_reply_t * : z_reply_drop,                                 \
                  z_owned_hello_t * : z_hello_drop,                                 \
                  z_owned_str_t * : z_str_drop,                                     \
                  z_owned_closure_sample_t * : z_closure_sample_drop,               \
                  z_owned_closure_query_t * : z_closure_query_drop,                 \
                  z_owned_closure_reply_t * : z_closure_reply_drop,                 \
                  z_owned_closure_hello_t * : z_closure_hello_drop,                 \
                  z_owned_closure_zid_t * : z_closure_zid_drop,                     \
                  z_owned_reply_channel_closure_t * : z_reply_channel_closure_drop, \
                  z_owned_reply_channel_t * : z_reply_channel_drop,                 \
                  zc_owned_payload_t * : zc_payload_drop,                           \
                  zc_owned_shmbuf_t * : zc_shmbuf_drop,                             \
                  zc_owned_shm_manager_t * : zc_shm_manager_drop                    \
            )(x)

#define z_null(x) (*x = \
    _Generic((x), z_owned_session_t * : z_session_null,                             \
                  z_owned_publisher_t * : z_publisher_null,                         \
                  z_owned_keyexpr_t * : z_keyexpr_null,                             \
                  z_owned_config_t * : z_config_null,                               \
                  z_owned_scouting_config_t * : z_scouting_config_null,             \
                  z_owned_pull_subscriber_t * : z_pull_subscriber_null,             \
                  z_owned_subscriber_t * : z_subscriber_null,                       \
                  z_owned_queryable_t * : z_queryable_null,                         \
                  z_owned_encoding_t * : z_encoding_null,                           \
                  z_owned_reply_t * : z_reply_null,                                 \
                  z_owned_hello_t * : z_hello_null,                                 \
                  z_owned_str_t * : z_str_null,                                     \
                  z_owned_closure_sample_t * : z_closure_sample_null,               \
                  z_owned_closure_query_t * : z_closure_query_null,                 \
                  z_owned_closure_reply_t * : z_closure_reply_null,                 \
                  z_owned_closure_hello_t * : z_closure_hello_null,                 \
                  z_owned_closure_zid_t * : z_closure_zid_null,                     \
                  z_owned_reply_channel_closure_t * : z_reply_channel_closure_null, \
                  z_owned_reply_channel_t * : z_reply_channel_null,                 \
                  zc_owned_payload_t * : zc_payload_null,                           \
                  zc_owned_shmbuf_t * : zc_shmbuf_null,                             \
                  zc_owned_shm_manager_t * : zc_shm_manager_null                    \
            )())

#define z_check(x) \
    _Generic((x), z_owned_session_t : z_session_check,                 \
                  z_owned_publisher_t : z_publisher_check,             \
                  z_owned_keyexpr_t : z_keyexpr_check,                 \
                  z_keyexpr_t : z_keyexpr_is_initialized,              \
                  z_owned_config_t : z_config_check,                   \
                  z_owned_scouting_config_t : z_scouting_config_check, \
                  z_bytes_t : z_bytes_check,                           \
                  z_owned_subscriber_t : z_subscriber_check,           \
                  z_owned_pull_subscriber_t : z_pull_subscriber_check, \
                  z_owned_queryable_t : z_queryable_check,             \
                  z_owned_encoding_t : z_encoding_check,               \
                  z_owned_reply_t : z_reply_check,                     \
                  z_owned_hello_t : z_hello_check,                     \
                  z_owned_str_t : z_str_check,                         \
                  zc_owned_payload_t : zc_payload_check,               \
                  zc_owned_shmbuf_t : zc_shmbuf_check,                 \
                  zc_owned_shm_manager_t : zc_shm_manager_check        \
            )(&x)

#define z_call(x, ...) \
    _Generic((x), z_owned_closure_sample_t : z_closure_sample_call,              \
                  z_owned_closure_query_t : z_closure_query_call,                \
                  z_owned_closure_reply_t : z_closure_reply_call,                \
                  z_owned_closure_hello_t : z_closure_hello_call,                \
                  z_owned_closure_zid_t : z_closure_zid_call,                    \
                  z_owned_reply_channel_closure_t : z_reply_channel_closure_call \
            ) (&x, __VA_ARGS__)
// clang-format on

#define _z_closure_overloader(callback, droper, ctx, ...) \
    { .context = (void*)ctx, .call = callback, .drop = droper }
#define z_closure(...) _z_closure_overloader(__VA_ARGS__, NULL, NULL)
#define z_move(x) (&x)

#else

// clang-format off
template<class T> struct zenoh_loan_type { typedef T type; };
template<class T> inline typename zenoh_loan_type<T>::type z_loan(const T&);

template<> struct zenoh_loan_type<z_owned_session_t>{ typedef z_session_t type; };
template<> struct zenoh_loan_type<z_owned_keyexpr_t>{ typedef z_keyexpr_t type; };
template<> struct zenoh_loan_type<z_owned_config_t>{ typedef z_config_t type; };
template<> struct zenoh_loan_type<z_owned_publisher_t>{ typedef z_publisher_t type; };
template<> struct zenoh_loan_type<z_owned_pull_subscriber_t>{ typedef z_pull_subscriber_t type; };
template<> struct zenoh_loan_type<z_owned_encoding_t>{ typedef z_encoding_t type; };
template<> struct zenoh_loan_type<z_owned_hello_t>{ typedef z_hello_t type; };
template<> struct zenoh_loan_type<z_owned_str_t>{  typedef const char* type; };

template<> inline z_session_t z_loan(const z_owned_session_t& x) { return z_session_loan(&x); }
template<> inline z_keyexpr_t z_loan(const z_owned_keyexpr_t& x) { return z_keyexpr_loan(&x); }
template<> inline z_config_t z_loan(const z_owned_config_t& x) { return z_config_loan(&x); }
template<> inline z_publisher_t z_loan(const z_owned_publisher_t& x) { return z_publisher_loan(&x); }
template<> inline z_pull_subscriber_t z_loan(const z_owned_pull_subscriber_t& x) { return z_pull_subscriber_loan(&x); }
template<> inline z_encoding_t z_loan(const z_owned_encoding_t& x) { return z_encoding_loan(&x); }
template<> inline z_hello_t z_loan(const z_owned_hello_t& x) { return z_hello_loan(&x); }
template<> inline const char* z_loan(const z_owned_str_t& x) { return z_str_loan(&x); }

template<class T> struct zenoh_drop_type { typedef T type; };
template<class T> inline typename zenoh_drop_type<T>::type z_drop(T*);

template<> struct zenoh_drop_type<z_owned_session_t> { typedef int8_t type; };
template<> struct zenoh_drop_type<z_owned_publisher_t> { typedef int8_t type; };
template<> struct zenoh_drop_type<z_owned_keyexpr_t> { typedef void type; };
template<> struct zenoh_drop_type<z_owned_config_t> { typedef void type; };
template<> struct zenoh_drop_type<z_owned_scouting_config_t> { typedef void type; };
template<> struct zenoh_drop_type<z_owned_pull_subscriber_t> { typedef int8_t type; };
template<> struct zenoh_drop_type<z_owned_subscriber_t> { typedef int8_t type; };
template<> struct zenoh_drop_type<z_owned_queryable_t> { typedef int8_t type; };
template<> struct zenoh_drop_type<z_owned_encoding_t> { typedef void type; };
template<> struct zenoh_drop_type<z_owned_reply_t> { typedef void type; };
template<> struct zenoh_drop_type<z_owned_hello_t> { typedef void type; };
template<> struct zenoh_drop_type<z_owned_str_t> { typedef void type; };
template<> struct zenoh_drop_type<zc_owned_payload_t> { typedef void type; };
template<> struct zenoh_drop_type<zc_owned_shmbuf_t> { typedef void type; };
template<> struct zenoh_drop_type<zc_owned_shm_manager_t> { typedef void type; };
template<> struct zenoh_drop_type<z_owned_closure_sample_t> { typedef void type; };
template<> struct zenoh_drop_type<z_owned_closure_query_t> { typedef void type; };
template<> struct zenoh_drop_type<z_owned_closure_reply_t> { typedef void type; };
template<> struct zenoh_drop_type<z_owned_closure_hello_t> { typedef void type; };
template<> struct zenoh_drop_type<z_owned_closure_zid_t> { typedef void type; };
template<> struct zenoh_drop_type<z_owned_reply_channel_closure_t> { typedef void type; };
template<> struct zenoh_drop_type<z_owned_reply_channel_t> { typedef void type; };

template<> inline int8_t z_drop(z_owned_session_t* v) { return z_close(v); }
template<> inline int8_t z_drop(z_owned_publisher_t* v) { return z_undeclare_publisher(v); }
template<> inline void z_drop(z_owned_keyexpr_t* v) { z_keyexpr_drop(v); }
template<> inline void z_drop(z_owned_config_t* v) { z_config_drop(v); }
template<> inline void z_drop(z_owned_scouting_config_t* v) { z_scouting_config_drop(v); }
template<> inline int8_t z_drop(z_owned_pull_subscriber_t* v) { return z_undeclare_pull_subscriber(v); }
template<> inline int8_t z_drop(z_owned_subscriber_t* v) { return z_undeclare_subscriber(v); }
template<> inline int8_t z_drop(z_owned_queryable_t* v) { return z_undeclare_queryable(v); }
template<> inline void z_drop(z_owned_encoding_t* v) { z_encoding_drop(v); }
template<> inline void z_drop(z_owned_reply_t* v) { z_reply_drop(v); }
template<> inline void z_drop(z_owned_hello_t* v) { z_hello_drop(v); }
template<> inline void z_drop(z_owned_str_t* v) { z_str_drop(v); }
template<> inline void z_drop(zc_owned_payload_t* v) { zc_payload_drop(v); }
template<> inline void z_drop(zc_owned_shmbuf_t* v) { zc_shmbuf_drop(v); }
template<> inline void z_drop(zc_owned_shm_manager_t* v) { zc_shm_manager_drop(v); }
template<> inline void z_drop(z_owned_closure_sample_t* v) { z_closure_sample_drop(v); }
template<> inline void z_drop(z_owned_closure_query_t* v) { z_closure_query_drop(v); }
template<> inline void z_drop(z_owned_closure_reply_t* v) { z_closure_reply_drop(v); }
template<> inline void z_drop(z_owned_closure_hello_t* v) { z_closure_hello_drop(v); }
template<> inline void z_drop(z_owned_closure_zid_t* v) { z_closure_zid_drop(v); }
template<> inline void z_drop(z_owned_reply_channel_closure_t* v) { z_reply_channel_closure_drop(v); }
template<> inline void z_drop(z_owned_reply_channel_t* v) { z_reply_channel_drop(v); }

inline void z_null(z_owned_session_t& v) { v = z_session_null(); }
inline void z_null(z_owned_publisher_t& v) { v = z_publisher_null(); }
inline void z_null(z_owned_keyexpr_t& v) { v = z_keyexpr_null(); }
inline void z_null(z_owned_config_t& v) { v = z_config_null(); }
inline void z_null(z_owned_scouting_config_t& v) { v = z_scouting_config_null(); }
inline void z_null(z_owned_pull_subscriber_t& v) { v = z_pull_subscriber_null(); }
inline void z_null(z_owned_subscriber_t& v) { v = z_subscriber_null(); }
inline void z_null(z_owned_queryable_t& v) { v = z_queryable_null(); }
inline void z_null(z_owned_encoding_t& v) { v = z_encoding_null(); }
inline void z_null(z_owned_reply_t& v) { v = z_reply_null(); }
inline void z_null(z_owned_hello_t& v) { v = z_hello_null(); }
inline void z_null(z_owned_str_t& v) { v = z_str_null(); }
inline void z_null(zc_owned_payload_t& v) { v = zc_payload_null(); }
inline void z_null(zc_owned_shmbuf_t& v) { v = zc_shmbuf_null(); }
inline void z_null(zc_owned_shm_manager_t& v) { v = zc_shm_manager_null(); }
inline void z_null(z_owned_closure_sample_t& v) { v = z_closure_sample_null(); }
inline void z_null(z_owned_closure_query_t& v) { v = z_closure_query_null(); }
inline void z_null(z_owned_closure_reply_t& v) { v = z_closure_reply_null(); }
inline void z_null(z_owned_closure_hello_t& v) { v = z_closure_hello_null(); }
inline void z_null(z_owned_closure_zid_t& v) { v = z_closure_zid_null(); }
inline void z_null(z_owned_reply_channel_closure_t& v) { v = z_reply_channel_closure_null(); }
inline void z_null(z_owned_reply_channel_t& v) { v = z_reply_channel_null(); }

inline bool z_check(const z_owned_session_t& v) { return z_session_check(&v); }
inline bool z_check(const z_owned_publisher_t& v) { return z_publisher_check(&v); }
inline bool z_check(const z_owned_keyexpr_t& v) { return z_keyexpr_check(&v); }
inline bool z_check(const z_keyexpr_t& v) { return z_keyexpr_is_initialized(&v); }
inline bool z_check(const z_owned_config_t& v) { return z_config_check(&v); }
inline bool z_check(const z_owned_scouting_config_t& v) { return z_scouting_config_check(&v); }
inline bool z_check(const z_bytes_t& v) { return z_bytes_check(&v); }
inline bool z_check(const zc_owned_payload_t& v) { return zc_payload_check(&v); }
inline bool z_check(const zc_owned_shmbuf_t& v) { return zc_shmbuf_check(&v); }
inline bool z_check(const zc_owned_shm_manager_t& v) { return zc_shm_manager_check(&v); }
inline bool z_check(const z_owned_subscriber_t& v) { return z_subscriber_check(&v); }
inline bool z_check(const z_owned_pull_subscriber_t& v) { return z_pull_subscriber_check(&v); }
inline bool z_check(const z_owned_queryable_t& v) { return z_queryable_check(&v); }
inline bool z_check(const z_owned_encoding_t& v) { return z_encoding_check(&v); }
inline bool z_check(const z_owned_reply_t& v) { return z_reply_check(&v); }
inline bool z_check(const z_owned_hello_t& v) { return z_hello_check(&v); }
inline bool z_check(const z_owned_str_t& v) { return z_str_check(&v); }

inline void z_call(const struct z_owned_closure_sample_t &closure, const struct z_sample_t *sample) 
    { z_closure_sample_call(&closure, sample); }
inline void z_call(const struct z_owned_closure_query_t &closure, const struct z_query_t *query)
    { z_closure_query_call(&closure, query); }
inline void z_call(const struct z_owned_closure_reply_t &closure, struct z_owned_reply_t *sample)
    { z_closure_reply_call(&closure, sample); }
inline void z_call(const struct z_owned_closure_hello_t &closure, struct z_owned_hello_t *hello)
    { z_closure_hello_call(&closure, hello); }
inline void z_call(const struct z_owned_closure_zid_t &closure, const struct z_id_t *zid)
    { z_closure_zid_call(&closure, zid); }
inline bool z_call(const struct z_owned_reply_channel_closure_t &closure, struct z_owned_reply_t *sample)
    { return z_reply_channel_closure_call(&closure, sample); }
// clang-format on

#define _z_closure_overloader(callback, droper, ctx, ...) \
    { .context = const_cast<void*>(static_cast<const void*>(ctx)), .call = callback, .drop = droper }
#define z_closure(...) _z_closure_overloader(__VA_ARGS__, NULL, NULL)
#define z_move(x) (&x)

#endif
