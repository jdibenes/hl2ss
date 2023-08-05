/**
 * The kind of congestion control.
 *
 *     - **BLOCK**
 *     - **DROP**
 */
typedef enum z_congestion_control_t {
  Z_CONGESTION_CONTROL_BLOCK,
  Z_CONGESTION_CONTROL_DROP,
} z_congestion_control_t;
/**
 * Consolidation mode values.
 *
 *     - **Z_CONSOLIDATION_MODE_AUTO**: Let Zenoh decide the best consolidation mode depending on the query selector
 *       If the selector contains time range properties, consolidation mode `NONE` is used.
 *       Otherwise the `LATEST` consolidation mode is used.
 *     - **Z_CONSOLIDATION_MODE_NONE**: No consolidation is applied. Replies may come in any order and any number.
 *     - **Z_CONSOLIDATION_MODE_MONOTONIC**: It guarantees that any reply for a given key expression will be monotonic in time
 *       w.r.t. the previous received replies for the same key expression. I.e., for the same key expression multiple
 *       replies may be received. It is guaranteed that two replies received at t1 and t2 will have timestamp
 *       ts2 > ts1. It optimizes latency.
 *     - **Z_CONSOLIDATION_MODE_LATEST**: It guarantees unicity of replies for the same key expression.
 *       It optimizes bandwidth.
 */
typedef enum z_consolidation_mode_t {
  Z_CONSOLIDATION_MODE_AUTO = -1,
  Z_CONSOLIDATION_MODE_NONE = 0,
  Z_CONSOLIDATION_MODE_MONOTONIC = 1,
  Z_CONSOLIDATION_MODE_LATEST = 2,
} z_consolidation_mode_t;
/**
 * A :c:type:`z_encoding_t` integer `prefix`.
 *
 *     - **Z_ENCODING_PREFIX_EMPTY**
 *     - **Z_ENCODING_PREFIX_APP_OCTET_STREAM**
 *     - **Z_ENCODING_PREFIX_APP_CUSTOM**
 *     - **Z_ENCODING_PREFIX_TEXT_PLAIN**
 *     - **Z_ENCODING_PREFIX_APP_PROPERTIES**
 *     - **Z_ENCODING_PREFIX_APP_JSON**
 *     - **Z_ENCODING_PREFIX_APP_SQL**
 *     - **Z_ENCODING_PREFIX_APP_INTEGER**
 *     - **Z_ENCODING_PREFIX_APP_FLOAT**
 *     - **Z_ENCODING_PREFIX_APP_XML**
 *     - **Z_ENCODING_PREFIX_APP_XHTML_XML**
 *     - **Z_ENCODING_PREFIX_APP_X_WWW_FORM_URLENCODED**
 *     - **Z_ENCODING_PREFIX_TEXT_JSON**
 *     - **Z_ENCODING_PREFIX_TEXT_HTML**
 *     - **Z_ENCODING_PREFIX_TEXT_XML**
 *     - **Z_ENCODING_PREFIX_TEXT_CSS**
 *     - **Z_ENCODING_PREFIX_TEXT_CSV**
 *     - **Z_ENCODING_PREFIX_TEXT_JAVASCRIPT**
 *     - **Z_ENCODING_PREFIX_IMAGE_JPEG**
 *     - **Z_ENCODING_PREFIX_IMAGE_PNG**
 *     - **Z_ENCODING_PREFIX_IMAGE_GIF**
 */
typedef enum z_encoding_prefix_t {
  Z_ENCODING_PREFIX_EMPTY = 0,
  Z_ENCODING_PREFIX_APP_OCTET_STREAM = 1,
  Z_ENCODING_PREFIX_APP_CUSTOM = 2,
  Z_ENCODING_PREFIX_TEXT_PLAIN = 3,
  Z_ENCODING_PREFIX_APP_PROPERTIES = 4,
  Z_ENCODING_PREFIX_APP_JSON = 5,
  Z_ENCODING_PREFIX_APP_SQL = 6,
  Z_ENCODING_PREFIX_APP_INTEGER = 7,
  Z_ENCODING_PREFIX_APP_FLOAT = 8,
  Z_ENCODING_PREFIX_APP_XML = 9,
  Z_ENCODING_PREFIX_APP_XHTML_XML = 10,
  Z_ENCODING_PREFIX_APP_X_WWW_FORM_URLENCODED = 11,
  Z_ENCODING_PREFIX_TEXT_JSON = 12,
  Z_ENCODING_PREFIX_TEXT_HTML = 13,
  Z_ENCODING_PREFIX_TEXT_XML = 14,
  Z_ENCODING_PREFIX_TEXT_CSS = 15,
  Z_ENCODING_PREFIX_TEXT_CSV = 16,
  Z_ENCODING_PREFIX_TEXT_JAVASCRIPT = 17,
  Z_ENCODING_PREFIX_IMAGE_JPEG = 18,
  Z_ENCODING_PREFIX_IMAGE_PNG = 19,
  Z_ENCODING_PREFIX_IMAGE_GIF = 20,
} z_encoding_prefix_t;
/**
 * The priority of zenoh messages.
 *
 *     - **REAL_TIME**
 *     - **INTERACTIVE_HIGH**
 *     - **INTERACTIVE_LOW**
 *     - **DATA_HIGH**
 *     - **DATA**
 *     - **DATA_LOW**
 *     - **BACKGROUND**
 */
typedef enum z_priority_t {
  Z_PRIORITY_REAL_TIME = 1,
  Z_PRIORITY_INTERACTIVE_HIGH = 2,
  Z_PRIORITY_INTERACTIVE_LOW = 3,
  Z_PRIORITY_DATA_HIGH = 4,
  Z_PRIORITY_DATA = 5,
  Z_PRIORITY_DATA_LOW = 6,
  Z_PRIORITY_BACKGROUND = 7,
} z_priority_t;
/**
 * The Queryables that should be target of a :c:func:`z_get`.
 *
 *     - **BEST_MATCHING**: The nearest complete queryable if any else all matching queryables.
 *     - **ALL_COMPLETE**: All complete queryables.
 *     - **ALL**: All matching queryables.
 */
typedef enum z_query_target_t {
  Z_QUERY_TARGET_BEST_MATCHING,
  Z_QUERY_TARGET_ALL,
  Z_QUERY_TARGET_ALL_COMPLETE,
} z_query_target_t;
/**
 * The subscription reliability.
 *
 *     - **Z_RELIABILITY_BEST_EFFORT**
 *     - **Z_RELIABILITY_RELIABLE**
 */
typedef enum z_reliability_t {
  Z_RELIABILITY_BEST_EFFORT,
  Z_RELIABILITY_RELIABLE,
} z_reliability_t;
typedef enum z_sample_kind_t {
  Z_SAMPLE_KIND_PUT = 0,
  Z_SAMPLE_KIND_DELETE = 1,
} z_sample_kind_t;
/**
 * An array of bytes.
 */
typedef struct z_bytes_t {
  size_t len;
  const uint8_t *start;
} z_bytes_t;
/**
 * Represents a Zenoh ID.
 *
 * In general, valid Zenoh IDs are LSB-first 128bit unsigned and non-zero integers.
 */
typedef struct z_id_t {
  uint8_t id[16];
} z_id_t;
/**
 * An owned array of owned, zenoh allocated, NULL terminated strings.
 *
 * Like all `z_owned_X_t`, an instance will be destroyed by any function which takes a mutable pointer to said instance, as this implies the instance's inners were moved.
 * To make this fact more obvious when reading your code, consider using `z_move(val)` instead of `&val` as the argument.
 * After a move, `val` will still exist, but will no longer be valid. The destructors are double-drop-safe, but other functions will still trust that your `val` is valid.
 *
 * To check if `val` is still valid, you may use `z_X_check(&val)` or `z_check(val)` if your compiler supports `_Generic`, which will return `true` if `val` is valid.
 */
typedef struct z_owned_str_array_t {
  char **val;
  size_t len;
} z_owned_str_array_t;
/**
 * A zenoh-allocated hello message returned by a zenoh entity to a scout message sent with `z_scout`.
 *
 * Members:
 *   unsigned int whatami: The kind of zenoh entity.
 *   z_owned_bytes_t pid: The peer id of the scouted entity (empty if absent).
 *   z_owned_str_array_t locators: The locators of the scouted entity.
 *
 * Like all `z_owned_X_t`, an instance will be destroyed by any function which takes a mutable pointer to said instance, as this implies the instance's inners were moved.
 * To make this fact more obvious when reading your code, consider using `z_move(val)` instead of `&val` as the argument.
 * After a move, `val` will still exist, but will no longer be valid. The destructors are double-drop-safe, but other functions will still trust that your `val` is valid.
 *
 * To check if `val` is still valid, you may use `z_X_check(&val)` (or `z_check(val)` if your compiler supports `_Generic`), which will return `true` if `val` is valid.
 */
typedef struct z_owned_hello_t {
  unsigned int _whatami;
  struct z_id_t _pid;
  struct z_owned_str_array_t _locators;
} z_owned_hello_t;
/**
 * A closure is a structure that contains all the elements for stateful, memory-leak-free callbacks:
 *
 * Members:
 *   void *context: a pointer to an arbitrary state.
 *   void *call(const struct z_owned_reply_t*, const void *context): the typical callback function. `context` will be passed as its last argument.
 *   void *drop(void*): allows the callback's state to be freed.
 *
 * Closures are not guaranteed not to be called concurrently.
 *
 * It is guaranteed that:
 *
 *   - `call` will never be called once `drop` has started.
 *   - `drop` will only be called **once**, and **after every** `call` has ended.
 *   - The two previous guarantees imply that `call` and `drop` are never called concurrently.
 */
typedef struct z_owned_closure_hello_t {
  void *context;
  void (*call)(struct z_owned_hello_t*, void*);
  void (*drop)(void*);
} z_owned_closure_hello_t;
/**
 * A closure is a structure that contains all the elements for stateful, memory-leak-free callbacks:
 *
 * Members:
 *   void *context: a pointer to an arbitrary state.
 *   void *call(const struct z_query_t*, const void *context): the typical callback function. `context` will be passed as its last argument.
 *   void *drop(void*): allows the callback's state to be freed.
 *
 * Closures are not guaranteed not to be called concurrently.
 *
 * It is guaranteed that:
 *
 *   - `call` will never be called once `drop` has started.
 *   - `drop` will only be called **once**, and **after every** `call` has ended.
 *   - The two previous guarantees imply that `call` and `drop` are never called concurrently.
 */
typedef struct z_owned_closure_query_t {
  void *context;
  void (*call)(const struct z_query_t*, void *context);
  void (*drop)(void*);
} z_owned_closure_query_t;
/**
 * An owned reply to a :c:func:`z_get`.
 *
 * Like all `z_owned_X_t`, an instance will be destroyed by any function which takes a mutable pointer to said instance, as this implies the instance's inners were moved.
 * To make this fact more obvious when reading your code, consider using `z_move(val)` instead of `&val` as the argument.
 * After a move, `val` will still exist, but will no longer be valid. The destructors are double-drop-safe, but other functions will still trust that your `val` is valid.
 *
 * To check if `val` is still valid, you may use `z_X_check(&val)` (or `z_check(val)` if your compiler supports `_Generic`), which will return `true` if `val` is valid.
 */
#if defined(TARGET_ARCH_X86_64)
typedef struct ALIGN(8) z_owned_reply_t {
  uint64_t _0[22];
} z_owned_reply_t;
#endif
#if defined(TARGET_ARCH_AARCH64)
typedef struct ALIGN(16) z_owned_reply_t {
  uint64_t _0[24];
} z_owned_reply_t;
#endif
#if defined(TARGET_ARCH_ARM)
typedef struct ALIGN(8) z_owned_reply_t {
  uint64_t _0[17];
} z_owned_reply_t;
#endif
/**
 * A closure is a structure that contains all the elements for stateful, memory-leak-free callbacks:
 *
 * Members:
 *   void *context: a pointer to an arbitrary state.
 *   void *call(const struct z_owned_reply_t*, const void *context): the typical callback function. `context` will be passed as its last argument.
 *   void *drop(void*): allows the callback's state to be freed.
 *
 * Closures are not guaranteed not to be called concurrently.
 *
 * It is guaranteed that:
 *
 *   - `call` will never be called once `drop` has started.
 *   - `drop` will only be called **once**, and **after every** `call` has ended.
 *   - The two previous guarantees imply that `call` and `drop` are never called concurrently.
 */
typedef struct z_owned_closure_reply_t {
  void *context;
  void (*call)(struct z_owned_reply_t*, void*);
  void (*drop)(void*);
} z_owned_closure_reply_t;
/**
 * A loaned key expression.
 *
 * Key expressions can identify a single key or a set of keys.
 *
 * Examples :
 *    - ``"key/expression"``.
 *    - ``"key/ex*"``.
 *
 * Using :c:func:`z_declare_keyexpr` allows zenoh to optimize a key expression,
 * both for local processing and network-wise.
 */
#if !defined(TARGET_ARCH_ARM)
typedef struct ALIGN(8) z_keyexpr_t {
  uint64_t _0[4];
} z_keyexpr_t;
#endif
#if defined(TARGET_ARCH_ARM)
typedef struct ALIGN(8) z_keyexpr_t {
  uint64_t _0[3];
} z_keyexpr_t;
#endif
/**
 * The encoding of a payload, in a MIME-like format.
 *
 * For wire and matching efficiency, common MIME types are represented using an integer as `prefix`, and a `suffix` may be used to either provide more detail, or in combination with the `Empty` prefix to write arbitrary MIME types.
 *
 * Members:
 *   z_encoding_prefix_t prefix: The integer prefix of this encoding.
 *   z_bytes_t suffix: The suffix of this encoding. `suffix` MUST be a valid UTF-8 string.
 */
typedef struct z_encoding_t {
  enum z_encoding_prefix_t prefix;
  struct z_bytes_t suffix;
} z_encoding_t;
typedef struct z_timestamp_t {
  uint64_t time;
  struct z_id_t id;
} z_timestamp_t;
/**
 * A data sample.
 *
 * A sample is the value associated to a given resource at a given point in time.
 *
 * Members:
 *   z_keyexpr_t keyexpr: The resource key of this data sample.
 *   z_bytes_t payload: The value of this data sample.
 *   z_encoding_t encoding: The encoding of the value of this data sample.
 *   z_sample_kind_t kind: The kind of this data sample (PUT or DELETE).
 *   z_timestamp_t timestamp: The timestamp of this data sample.
 */
typedef struct z_sample_t {
  struct z_keyexpr_t keyexpr;
  struct z_bytes_t payload;
  struct z_encoding_t encoding;
  const void *_zc_buf;
  enum z_sample_kind_t kind;
  struct z_timestamp_t timestamp;
} z_sample_t;
/**
 * A closure is a structure that contains all the elements for stateful, memory-leak-free callbacks.
 *
 * Members:
 *   void *context: a pointer to an arbitrary state.
 *   void *call(const struct z_sample_t*, const void *context): the typical callback function. `context` will be passed as its last argument.
 *   void *drop(void*): allows the callback's state to be freed.
 *
 * Closures are not guaranteed not to be called concurrently.
 *
 * It is guaranteed that:
 *
 *   - `call` will never be called once `drop` has started.
 *   - `drop` will only be called **once**, and **after every** `call` has ended.
 *   - The two previous guarantees imply that `call` and `drop` are never called concurrently.
 */
typedef struct z_owned_closure_sample_t {
  void *context;
  void (*call)(const struct z_sample_t*, void *context);
  void (*drop)(void*);
} z_owned_closure_sample_t;
/**
 * A closure is a structure that contains all the elements for stateful, memory-leak-free callbacks:
 *
 * Members:
 *   void *context: a pointer to an arbitrary state.
 *   void *call(const struct z_owned_reply_t*, const void *context): the typical callback function. `context` will be passed as its last argument.
 *   void *drop(void*): allows the callback's state to be freed.
 *
 * Closures are not guaranteed not to be called concurrently.
 *
 * It is guaranteed that:
 *
 *   - `call` will never be called once `drop` has started.
 *   - `drop` will only be called **once**, and **after every** `call` has ended.
 *   - The two previous guarantees imply that `call` and `drop` are never called concurrently.
 */
typedef struct z_owned_closure_zid_t {
  void *context;
  void (*call)(const struct z_id_t*, void*);
  void (*drop)(void*);
} z_owned_closure_zid_t;
/**
 * An owned zenoh configuration.
 *
 * Like most `z_owned_X_t` types, you may obtain an instance of `z_X_t` by loaning it using `z_X_loan(&val)`.
 * The `z_loan(val)` macro, available if your compiler supports C11's `_Generic`, is equivalent to writing `z_X_loan(&val)`.
 *
 * Like all `z_owned_X_t`, an instance will be destroyed by any function which takes a mutable pointer to said instance, as this implies the instance's inners were moved.
 * To make this fact more obvious when reading your code, consider using `z_move(val)` instead of `&val` as the argument.
 * After a move, `val` will still exist, but will no longer be valid. The destructors are double-drop-safe, but other functions will still trust that your `val` is valid.
 *
 * To check if `val` is still valid, you may use `z_X_check(&val)` or `z_check(val)` if your compiler supports `_Generic`, which will return `true` if `val` is valid.
 */
typedef struct z_owned_config_t {
  void *_0;
} z_owned_config_t;
/**
 * A loaned zenoh configuration.
 */
typedef struct z_config_t {
  const struct z_owned_config_t *_0;
} z_config_t;
/**
 * A zenoh-allocated key expression.
 *
 * Key expressions can identify a single key or a set of keys.
 *
 * Examples :
 *    - ``"key/expression"``.
 *    - ``"key/ex*"``.
 *
 * Key expressions can be mapped to numerical ids through :c:func:`z_declare_expr`
 * for wire and computation efficiency.
 *
 * A `key expression <https://github.com/eclipse-zenoh/roadmap/blob/main/rfcs/ALL/Key%20Expressions.md>`_ can be either:
 *   - A plain string expression.
 *   - A pure numerical id.
 *   - The combination of a numerical prefix and a string suffix.
 *
 * Like most `z_owned_X_t` types, you may obtain an instance of `z_X_t` by loaning it using `z_X_loan(&val)`.
 * The `z_loan(val)` macro, available if your compiler supports C11's `_Generic`, is equivalent to writing `z_X_loan(&val)`.
 *
 * Like all `z_owned_X_t`, an instance will be destroyed by any function which takes a mutable pointer to said instance, as this implies the instance's inners were moved.
 * To make this fact more obvious when reading your code, consider using `z_move(val)` instead of `&val` as the argument.
 * After a move, `val` will still exist, but will no longer be valid. The destructors are double-drop-safe, but other functions will still trust that your `val` is valid.
 *
 * To check if `val` is still valid, you may use `z_X_check(&val)` or `z_check(val)` if your compiler supports `_Generic`, which will return `true` if `val` is valid.
 */
#if !defined(TARGET_ARCH_ARM)
typedef struct ALIGN(8) z_owned_keyexpr_t {
  uint64_t _0[4];
} z_owned_keyexpr_t;
#endif
#if defined(TARGET_ARCH_ARM)
typedef struct ALIGN(8) z_owned_keyexpr_t {
  uint64_t _0[3];
} z_owned_keyexpr_t;
#endif
/**
 * An owned zenoh publisher.
 *
 * Like most `z_owned_X_t` types, you may obtain an instance of `z_X_t` by loaning it using `z_X_loan(&val)`.
 * The `z_loan(val)` macro, available if your compiler supports C11's `_Generic`, is equivalent to writing `z_X_loan(&val)`.
 *
 * Like all `z_owned_X_t`, an instance will be destroyed by any function which takes a mutable pointer to said instance, as this implies the instance's inners were moved.
 * To make this fact more obvious when reading your code, consider using `z_move(val)` instead of `&val` as the argument.
 * After a move, `val` will still exist, but will no longer be valid. The destructors are double-drop-safe, but other functions will still trust that your `val` is valid.
 *
 * To check if `val` is still valid, you may use `z_X_check(&val)` or `z_check(val)` if your compiler supports `_Generic`, which will return `true` if `val` is valid.
 */
#if !defined(TARGET_ARCH_ARM)
typedef struct ALIGN(8) z_owned_publisher_t {
  uint64_t _0[7];
} z_owned_publisher_t;
#endif
#if defined(TARGET_ARCH_ARM)
typedef struct ALIGN(8) z_owned_publisher_t {
  uint64_t _0[5];
} z_owned_publisher_t;
#endif
/**
 * Options passed to the :c:func:`z_declare_publisher` function.
 *
 * Members:
 *     z_congestion_control_t congestion_control: The congestion control to apply when routing messages from this publisher.
 *     z_priority_t priority: The priority of messages from this publisher.
 */
typedef struct z_publisher_options_t {
  enum z_congestion_control_t congestion_control;
  enum z_priority_t priority;
} z_publisher_options_t;
/**
 * Represents the set of options that can be applied to a pull subscriber,
 * upon its declaration via :c:func:`z_declare_pull_subscriber`.
 *
 * Members:
 *   z_reliability_t reliability: The subscription reliability.
 */
typedef struct z_pull_subscriber_options_t {
  enum z_reliability_t reliability;
} z_pull_subscriber_options_t;
/**
 * Options passed to the :c:func:`z_declare_queryable` function.
 *
 * Members:
 *     bool complete: The completeness of the Queryable.
 */
typedef struct z_queryable_options_t {
  bool complete;
} z_queryable_options_t;
/**
 * Options passed to the :c:func:`z_declare_subscriber` or :c:func:`z_declare_pull_subscriber` function.
 *
 * Members:
 *     z_reliability_t reliability: The subscription reliability.
 */
typedef struct z_subscriber_options_t {
  enum z_reliability_t reliability;
} z_subscriber_options_t;
/**
 * Options passed to the :c:func:`z_delete` function.
 */
typedef struct z_delete_options_t {
  enum z_congestion_control_t congestion_control;
  enum z_priority_t priority;
} z_delete_options_t;
/**
 * An owned payload encoding.
 *
 * Members:
 *   z_encoding_prefix_t prefix: The integer prefix of this encoding.
 *   z_bytes_t suffix: The suffix of this encoding. `suffix` MUST be a valid UTF-8 string.
 *
 * Like all `z_owned_X_t`, an instance will be destroyed by any function which takes a mutable pointer to said instance, as this implies the instance's inners were moved.
 * To make this fact more obvious when reading your code, consider using `z_move(val)` instead of `&val` as the argument.
 * After a move, `val` will still exist, but will no longer be valid. The destructors are double-drop-safe, but other functions will still trust that your `val` is valid.
 *
 * To check if `val` is still valid, you may use `z_X_check(&val)` (or `z_check(val)` if your compiler supports `_Generic`), which will return `true` if `val` is valid.
 */
typedef struct z_owned_encoding_t {
  enum z_encoding_prefix_t prefix;
  struct z_bytes_t suffix;
  bool _dropped;
} z_owned_encoding_t;
/**
 * The replies consolidation strategy to apply on replies to a :c:func:`z_get`.
 */
typedef struct z_query_consolidation_t {
  enum z_consolidation_mode_t mode;
} z_query_consolidation_t;
/**
 * A zenoh value.
 *
 * Members:
 *   z_bytes_t payload: The payload of this zenoh value.
 *   z_encoding_t encoding: The encoding of this zenoh value `payload`.
 */
typedef struct z_value_t {
  struct z_bytes_t payload;
  struct z_encoding_t encoding;
} z_value_t;
/**
 * Options passed to the :c:func:`z_get` function.
 *
 * Members:
 *     z_query_target_t target: The Queryables that should be target of the query.
 *     z_query_consolidation_t consolidation: The replies consolidation strategy to apply on replies to the query.
 *     z_value_t value: An optional value to attach to the query.
 */
typedef struct z_get_options_t {
  enum z_query_target_t target;
  struct z_query_consolidation_t consolidation;
  struct z_value_t value;
} z_get_options_t;
/**
 * An borrowed array of borrowed, zenoh allocated, NULL terminated strings.
 */
typedef struct z_str_array_t {
  size_t len;
  const char *const *val;
} z_str_array_t;
/**
 * A reference-type hello message returned by a zenoh entity to a scout message sent with `z_scout`.
 *
 * Members:
 *   unsigned int whatami: The kind of zenoh entity.
 *   z_owned_bytes_t pid: The peer id of the scouted entity (empty if absent).
 *   z_owned_str_array_t locators: The locators of the scouted entity.
 *
 * Like all `z_owned_X_t`, an instance will be destroyed by any function which takes a mutable pointer to said instance, as this implies the instance's inners were moved.
 * To make this fact more obvious when reading your code, consider using `z_move(val)` instead of `&val` as the argument.
 * After a move, `val` will still exist, but will no longer be valid. The destructors are double-drop-safe, but other functions will still trust that your `val` is valid.
 *
 * To check if `val` is still valid, you may use `z_X_check(&val)` (or `z_check(val)` if your compiler supports `_Generic`), which will return `true` if `val` is valid.
 */
typedef struct z_hello_t {
  unsigned int whatami;
  struct z_id_t pid;
  struct z_str_array_t locators;
} z_hello_t;
/**
 * The wrapper type for null-terminated string values allocated by zenoh. The instances of `z_owned_str_t`
 * should be released with `z_drop` macro or with `z_str_drop` function and checked to validity with
 * `z_check` and `z_str_check` correspondently
 */
typedef struct z_owned_str_t {
  char *_cstr;
} z_owned_str_t;
/**
 * A loaned zenoh publisher.
 */
typedef struct z_publisher_t {
  const struct z_owned_publisher_t *_0;
} z_publisher_t;
/**
 * Represents the set of options that can be applied to the delete operation by a previously declared publisher,
 * whenever issued via :c:func:`z_publisher_delete`.
 */
typedef struct z_publisher_delete_options_t {
  uint8_t __dummy;
} z_publisher_delete_options_t;
/**
 * Options passed to the :c:func:`z_publisher_put` function.
 *
 * Members:
 *     z_encoding_t encoding: The encoding of the payload.
 */
typedef struct z_publisher_put_options_t {
  struct z_encoding_t encoding;
} z_publisher_put_options_t;
typedef struct z_pull_subscriber_t {
  const struct z_owned_pull_subscriber_t *_0;
} z_pull_subscriber_t;
/**
 * Options passed to the :c:func:`z_put` function.
 *
 * Members:
 *     z_encoding_t encoding: The encoding of the payload.
 *     z_congestion_control_t congestion_control: The congestion control to apply when routing this message.
 *     z_priority_t priority: The priority of this message.
 */
typedef struct z_put_options_t {
  struct z_encoding_t encoding;
  enum z_congestion_control_t congestion_control;
  enum z_priority_t priority;
} z_put_options_t;
/**
 * Represents the set of options that can be applied to a query reply,
 * sent via :c:func:`z_query_reply`.
 *
 * Members:
 *   z_encoding_t encoding: The encoding of the payload.
 */
typedef struct z_query_reply_options_t {
  struct z_encoding_t encoding;
} z_query_reply_options_t;
/**
 * A closure is a structure that contains all the elements for stateful, memory-leak-free callbacks:
 * - `this` is a pointer to an arbitrary state.
 * - `call` is the typical callback function. `this` will be passed as its last argument.
 * - `drop` allows the callback's state to be freed.
 *
 * Closures are not guaranteed not to be called concurrently.
 *
 * We guarantee that:
 * - `call` will never be called once `drop` has started.
 * - `drop` will only be called ONCE, and AFTER EVERY `call` has ended.
 * - The two previous guarantees imply that `call` and `drop` are never called concurrently.
 */
typedef struct z_owned_reply_channel_closure_t {
  void *context;
  bool (*call)(struct z_owned_reply_t*, void*);
  void (*drop)(void*);
} z_owned_reply_channel_closure_t;
/**
 * A pair of closures, the `send` one accepting
 */
typedef struct z_owned_reply_channel_t {
  struct z_owned_closure_reply_t send;
  struct z_owned_reply_channel_closure_t recv;
} z_owned_reply_channel_t;
typedef struct z_owned_scouting_config_t {
  struct z_owned_config_t _config;
  unsigned long zc_timeout_ms;
  unsigned int zc_what;
} z_owned_scouting_config_t;
/**
 * A loaned zenoh subscriber.
 */
typedef struct z_subscriber_t {
  const struct z_owned_subscriber_t *_0;
} z_subscriber_t;
/**
 * An owned payload, backed by a reference counted owner.
 *
 * The `payload` field may be modified, and Zenoh will take the new values into account,
 * however, assuming `ostart` and `olen` are the respective values of `payload.start` and
 * `payload.len` when constructing the `zc_owned_payload_t payload` value was created,
 * then `payload.start` MUST remain within the `[ostart, ostart + olen[` interval, and
 * `payload.len` must remain within `[0, olen -(payload.start - ostart)]`.
 *
 * Should this invariant be broken when the payload is passed to one of zenoh's `put_owned`
 * functions, then the operation will fail (but the passed value will still be consumed).
 */
typedef struct zc_owned_payload_t {
  struct z_bytes_t payload;
  uintptr_t _owner[4];
} zc_owned_payload_t;
typedef struct zc_owned_shmbuf_t {
  uintptr_t _0[9];
} zc_owned_shmbuf_t;
typedef struct zc_owned_shm_manager_t {
  uintptr_t _0;
} zc_owned_shm_manager_t;
extern const unsigned int Z_ROUTER;
extern const unsigned int Z_PEER;
extern const unsigned int Z_CLIENT;
extern const char *Z_CONFIG_MODE_KEY;
extern const char *Z_CONFIG_CONNECT_KEY;
extern const char *Z_CONFIG_LISTEN_KEY;
extern const char *Z_CONFIG_USER_KEY;
extern const char *Z_CONFIG_PASSWORD_KEY;
extern const char *Z_CONFIG_MULTICAST_SCOUTING_KEY;
extern const char *Z_CONFIG_MULTICAST_INTERFACE_KEY;
extern const char *Z_CONFIG_MULTICAST_IPV4_ADDRESS_KEY;
extern const char *Z_CONFIG_SCOUTING_TIMEOUT_KEY;
extern const char *Z_CONFIG_SCOUTING_DELAY_KEY;
extern const char *Z_CONFIG_ADD_TIMESTAMP_KEY;
/**
 * Returns ``true`` if `b` is initialized.
 */
bool z_bytes_check(const struct z_bytes_t *b);
/**
 * Closes a zenoh session. This drops and invalidates `session` for double-drop safety.
 *
 * Returns a negative value if an error occured while closing the session.
 * Returns the remaining reference count of the session otherwise, saturating at i8::MAX.
 */
int8_t z_close(struct z_owned_session_t *session);
/**
 * Calls the closure. Calling an uninitialized closure is a no-op.
 */
void z_closure_hello_call(const struct z_owned_closure_hello_t *closure,
                          struct z_owned_hello_t *hello);
/**
 * Drops the closure. Droping an uninitialized closure is a no-op.
 */
void z_closure_hello_drop(struct z_owned_closure_hello_t *closure);
/**
 * Constructs a null safe-to-drop value of 'z_owned_closure_hello_t' type
 */
struct z_owned_closure_hello_t z_closure_hello_null(void);
/**
 * Calls the closure. Calling an uninitialized closure is a no-op.
 */
void z_closure_query_call(const struct z_owned_closure_query_t *closure,
                          const struct z_query_t *query);
/**
 * Drops the closure. Droping an uninitialized closure is a no-op.
 */
void z_closure_query_drop(struct z_owned_closure_query_t *closure);
/**
 * Constructs a null safe-to-drop value of 'z_owned_closure_query_t' type
 */
struct z_owned_closure_query_t z_closure_query_null(void);
/**
 * Calls the closure. Calling an uninitialized closure is a no-op.
 */
void z_closure_reply_call(const struct z_owned_closure_reply_t *closure,
                          struct z_owned_reply_t *sample);
/**
 * Drops the closure. Droping an uninitialized closure is a no-op.
 */
void z_closure_reply_drop(struct z_owned_closure_reply_t *closure);
/**
 * Constructs a null safe-to-drop value of 'z_owned_closure_reply_t' type
 */
struct z_owned_closure_reply_t z_closure_reply_null(void);
/**
 * Calls the closure. Calling an uninitialized closure is a no-op.
 */
void z_closure_sample_call(const struct z_owned_closure_sample_t *closure,
                           const struct z_sample_t *sample);
/**
 * Drops the closure. Droping an uninitialized closure is a no-op.
 */
void z_closure_sample_drop(struct z_owned_closure_sample_t *closure);
/**
 * Constructs a null safe-to-drop value of 'z_owned_closure_sample_t' type
 */
struct z_owned_closure_sample_t z_closure_sample_null(void);
/**
 * Calls the closure. Calling an uninitialized closure is a no-op.
 */
void z_closure_zid_call(const struct z_owned_closure_zid_t *closure, const struct z_id_t *sample);
/**
 * Drops the closure. Droping an uninitialized closure is a no-op.
 */
void z_closure_zid_drop(struct z_owned_closure_zid_t *closure);
/**
 * Constructs a null safe-to-drop value of 'z_owned_closure_zid_t' type
 */
struct z_owned_closure_zid_t z_closure_zid_null(void);
/**
 * Returns ``true`` if `config` is valid.
 */
bool z_config_check(const struct z_owned_config_t *config);
/**
 * Constructs a default, zenoh-allocated, client mode configuration.
 * If `peer` is not null, it is added to the configuration as remote peer.
 */
struct z_owned_config_t z_config_client(const char *const *peers, uintptr_t n_peers);
/**
 * Creates a default, zenoh-allocated, configuration.
 */
struct z_owned_config_t z_config_default(void);
/**
 * Frees `config`, invalidating it for double-drop safety.
 */
void z_config_drop(struct z_owned_config_t *config);
/**
 * Returns a :c:type:`z_config_t` loaned from `s`.
 */
struct z_config_t z_config_loan(const struct z_owned_config_t *s);
/**
 * Return a new, zenoh-allocated, empty configuration.
 *
 * Like most `z_owned_X_t` types, you may obtain an instance of `z_X_t` by loaning it using `z_X_loan(&val)`.
 * The `z_loan(val)` macro, available if your compiler supports C11's `_Generic`, is equivalent to writing `z_X_loan(&val)`.
 *
 * Like all `z_owned_X_t`, an instance will be destroyed by any function which takes a mutable pointer to said instance, as this implies the instance's inners were moved.
 * To make this fact more obvious when reading your code, consider using `z_move(val)` instead of `&val` as the argument.
 * After a move, `val` will still exist, but will no longer be valid. The destructors are double-drop-safe, but other functions will still trust that your `val` is valid.
 *
 * To check if `val` is still valid, you may use `z_X_check(&val)` or `z_check(val)` if your compiler supports `_Generic`, which will return `true` if `val` is valid.
 */
struct z_owned_config_t z_config_new(void);
/**
 * Constructs a null safe-to-drop value of 'z_owned_config_t' type
 */
struct z_owned_config_t z_config_null(void);
/**
 * Constructs a default, zenoh-allocated, peer mode configuration.
 */
struct z_owned_config_t z_config_peer(void);
/**
 * Declare a key expression. The id is returned as a :c:type:`z_keyexpr_t` with a nullptr suffix.
 *
 * This numerical id will be used on the network to save bandwidth and
 * ease the retrieval of the concerned resource in the routing tables.
 */
struct z_owned_keyexpr_t z_declare_keyexpr(struct z_session_t session, struct z_keyexpr_t keyexpr);
/**
 * Declares a publisher for the given key expression.
 *
 * Data can be put and deleted with this publisher with the help of the
 * :c:func:`z_publisher_put` and :c:func:`z_publisher_delete` functions.
 *
 * Parameters:
 *     session: The zenoh session.
 *     keyexpr: The key expression to publish.
 *     options: additional options for the publisher.
 *
 * Returns:
 *    A :c:type:`z_owned_publisherr_t`.
 *
 *    To check if the publisher decalration succeeded and if the publisher is still valid,
 *    you may use `z_publisher_check(&val)` or `z_check(val)` if your compiler supports `_Generic`, which will return `true` if `val` is valid.
 *
 *    Like all `z_owned_X_t`, an instance will be destroyed by any function which takes a mutable pointer to said instance, as this implies the instance's inners were moved.
 *    To make this fact more obvious when reading your code, consider using `z_move(val)` instead of `&val` as the argument.
 *    After a move, `val` will still exist, but will no longer be valid. The destructors are double-drop-safe, but other functions will still trust that your `val` is valid.
 *
 * Example:
 *    Declaring a publisher passing `NULL` for the options:
 *
 *    .. code-block:: C
 *
 *       z_owned_publisher_t pub = z_declare_publisher(z_loan(s), z_keyexpr(expr), NULL);
 *
 *    is equivalent to initializing and passing the default publisher options:
 *
 *    .. code-block:: C
 *
 *       z_publisher_options_t opts = z_publisher_options_default();
 *       z_owned_publisher_t sub = z_declare_publisher(z_loan(s), z_keyexpr(expr), &opts);
 */
struct z_owned_publisher_t z_declare_publisher(struct z_session_t session,
                                               struct z_keyexpr_t keyexpr,
                                               const struct z_publisher_options_t *options);
/**
 * Declares a pull subscriber for a given key expression.
 *
 * Parameters:
 *     session: The zenoh session.
 *     keyexpr: The key expression to subscribe.
 *     callback: The callback function that will be called each time a data matching the subscribed expression is received.
 *     opts: additional options for the pull subscriber.
 *
 * Returns:
 *    A :c:type:`z_owned_subscriber_t`.
 *
 *    To check if the subscription succeeded and if the pull subscriber is still valid,
 *    you may use `z_pull_subscriber_check(&val)` or `z_check(val)` if your compiler supports `_Generic`, which will return `true` if `val` is valid.
 *
 *    Like all `z_owned_X_t`, an instance will be destroyed by any function which takes a mutable pointer to said instance, as this implies the instance's inners were moved.
 *    To make this fact more obvious when reading your code, consider using `z_move(val)` instead of `&val` as the argument.
 *    After a move, `val` will still exist, but will no longer be valid. The destructors are double-drop-safe, but other functions will still trust that your `val` is valid.
 *
 * Example:
 *    Declaring a subscriber passing ``NULL`` for the options:
 *
 *    .. code-block:: C
 *
 *       z_owned_subscriber_t sub = z_declare_pull_subscriber(z_loan(s), z_keyexpr(expr), callback, NULL);
 *
 *    is equivalent to initializing and passing the default subscriber options:
 *
 *    .. code-block:: C
 *
 *       z_subscriber_options_t opts = z_subscriber_options_default();
 *       z_owned_subscriber_t sub = z_declare_pull_subscriber(z_loan(s), z_keyexpr(expr), callback, &opts);
 *
 *    Passing custom arguments to the **callback** can be done by defining a custom structure:
 *
 *    .. code-block:: C
 *
 *       typedef struct {
 *         z_keyexpr_t forward;
 *         z_session_t session;
 *       } myargs_t;
 *
 *       void callback(const z_sample_t sample, const void *arg)
 *       {
 *         myargs_t *myargs = (myargs_t *)arg;
 *         z_put(myargs->session, myargs->forward, sample->value, NULL);
 *       }
 *
 *       int main() {
 *         myargs_t cargs = {
 *           forward = z_keyexpr("forward"),
 *           session = s,
 *         };
 *         z_pull_subscriber_options_t opts = z_pull_subscriber_options_default();
 *         opts.cargs = (void *)&cargs;
 *         z_owned_pull_subscriber_t sub = z_declare_pull_subscriber(z_loan(s), z_keyexpr(expr), callback, &opts);
 *       }
 */
struct z_owned_pull_subscriber_t z_declare_pull_subscriber(struct z_session_t session,
                                                           struct z_keyexpr_t keyexpr,
                                                           struct z_owned_closure_sample_t *callback,
                                                           const struct z_pull_subscriber_options_t *opts);
/**
 * Creates a Queryable for the given key expression.
 *
 * Parameters:
 *     session: The zenoh session.
 *     keyexpr: The key expression the Queryable will reply to.
 *     callback: The callback function that will be called each time a matching query is received.
 *     options: Options for the queryable.
 *
 * Returns:
 *    The created :c:type:`z_owned_queryable_t` or ``null`` if the creation failed.
 */
struct z_owned_queryable_t z_declare_queryable(struct z_session_t session,
                                               struct z_keyexpr_t keyexpr,
                                               struct z_owned_closure_query_t *callback,
                                               const struct z_queryable_options_t *options);
/**
 * Declare a subscriber for a given key expression.
 *
 * Parameters:
 *     session: The zenoh session.
 *     keyexpr: The key expression to subscribe.
 *     callback: The callback function that will be called each time a data matching the subscribed expression is received.
 *     opts: The options to be passed to describe the options to be passed to the subscriber declaration.
 *
 * Returns:
 *    A :c:type:`z_owned_subscriber_t`.
 *
 *    To check if the subscription succeeded and if the subscriber is still valid,
 *    you may use `z_subscriber_check(&val)` or `z_check(val)` if your compiler supports `_Generic`, which will return `true` if `val` is valid.
 *
 *    Like all `z_owned_X_t`, an instance will be destroyed by any function which takes a mutable pointer to said instance, as this implies the instance's inners were moved.
 *    To make this fact more obvious when reading your code, consider using `z_move(val)` instead of `&val` as the argument.
 *    After a move, `val` will still exist, but will no longer be valid. The destructors are double-drop-safe, but other functions will still trust that your `val` is valid.
 *
 * Example:
 *    Declaring a subscriber passing `NULL` for the options:
 *
 *    .. code-block:: C
 *
 *       z_owned_subscriber_t sub = z_declare_subscriber(z_loan(s), z_keyexpr(expr), callback, NULL);
 *
 *    is equivalent to initializing and passing the default subscriber options:
 *
 *    .. code-block:: C
 *
 *       z_subscriber_options_t opts = z_subscriber_options_default();
 *       z_owned_subscriber_t sub = z_declare_subscriber(z_loan(s), z_keyexpr(expr), callback, &opts);
 *
 *    Passing custom arguments to the **callback** can be done by defining a custom structure:
 *
 *    .. code-block:: C
 *
 *       typedef struct {
 *         z_keyexpr_t forward;
 *         z_session_t session;
 *       } myargs_t;
 *
 *       void callback(const z_sample_t sample, const void *arg)
 *       {
 *         myargs_t *myargs = (myargs_t *)arg;
 *         z_put(myargs->session, myargs->forward, sample->value, NULL);
 *       }
 *
 *       int main() {
 *         myargs_t cargs = {
 *           forward = z_keyexpr("forward"),
 *           session = s,
 *         };
 *         z_subscriber_options_t opts = z_subscriber_options_default();
 *         opts.cargs = (void *)&cargs;
 *         z_owned_subscriber_t sub = z_declare_subscriber(z_loan(s), z_keyexpr(expr), callback, &opts);
 *       }
 */
struct z_owned_subscriber_t z_declare_subscriber(struct z_session_t session,
                                                 struct z_keyexpr_t keyexpr,
                                                 struct z_owned_closure_sample_t *callback,
                                                 const struct z_subscriber_options_t *opts);
/**
 * Delete data.
 *
 * Parameters:
 *     session: The zenoh session.
 *     keyexpr: The key expression to delete.
 *     options: The put options.
 * Returns:
 *     ``0`` in case of success, negative values in case of failure.
 */
int8_t z_delete(struct z_session_t session,
                struct z_keyexpr_t keyexpr,
                const struct z_delete_options_t *opts);
/**
 * Constructs the default value for :c:type:`z_put_options_t`.
 */
struct z_delete_options_t z_delete_options_default(void);
/**
 * Constructs a specific :c:type:`z_encoding_t`.
 */
struct z_encoding_t z_encoding(enum z_encoding_prefix_t prefix, const char *suffix);
/**
 * Returns ``true`` if `encoding` is valid.
 */
bool z_encoding_check(const struct z_owned_encoding_t *encoding);
/**
 * Constructs a default :c:type:`z_encoding_t`.
 */
struct z_encoding_t z_encoding_default(void);
/**
 * Frees `encoding`, invalidating it for double-drop safety.
 */
void z_encoding_drop(struct z_owned_encoding_t *encoding);
/**
 * Returns a :c:type:`z_encoding_t` loaned from `encoding`.
 */
struct z_encoding_t z_encoding_loan(const struct z_owned_encoding_t *encoding);
/**
 * Constructs a null safe-to-drop value of 'z_owned_encoding_t' type
 */
struct z_owned_encoding_t z_encoding_null(void);
/**
 * Query data from the matching queryables in the system.
 * Replies are provided through a callback function.
 *
 * Returns a negative value upon failure.
 *
 * Parameters:
 *     session: The zenoh session.
 *     keyexpr: The key expression matching resources to query.
 *     parameters: The query's parameters, similar to a url's query segment.
 *     callback: The callback function that will be called on reception of replies for this query.
 *               Note that the `reply` parameter of the callback is passed by mutable reference,
 *               but **will** be dropped once your callback exits to help you avoid memory leaks.
 *               If you'd rather take ownership, please refer to the documentation of :c:func:`z_reply_null`
 *     options: additional options for the get.
 */
int8_t z_get(struct z_session_t session,
             struct z_keyexpr_t keyexpr,
             const char *parameters,
             struct z_owned_closure_reply_t *callback,
             const struct z_get_options_t *options);
struct z_get_options_t z_get_options_default(void);
/**
 * Returns ``true`` if `hello` is valid.
 */
bool z_hello_check(const struct z_owned_hello_t *hello);
/**
 * Frees `hello`, invalidating it for double-drop safety.
 */
void z_hello_drop(struct z_owned_hello_t *hello);
/**
 * Returns a :c:type:`z_hello_t` loaned from :c:type:`z_owned_hello_t`.
 */
struct z_hello_t z_hello_loan(const struct z_owned_hello_t *hello);
/**
 * Constructs a gravestone value for hello, useful to steal one from a callback
 */
struct z_owned_hello_t z_hello_null(void);
/**
 * Fetches the Zenoh IDs of all connected peers.
 *
 * `callback` will be called once for each ID, is guaranteed to never be called concurrently,
 * and is guaranteed to be dropped before this function exits.
 *
 * Retuns 0 on success, negative values on failure
 */
int8_t z_info_peers_zid(struct z_session_t session, struct z_owned_closure_zid_t *callback);
/**
 * Fetches the Zenoh IDs of all connected routers.
 *
 * `callback` will be called once for each ID, is guaranteed to never be called concurrently,
 * and is guaranteed to be dropped before this function exits.
 *
 * Retuns 0 on success, negative values on failure.
 */
int8_t z_info_routers_zid(struct z_session_t session, struct z_owned_closure_zid_t *callback);
/**
 * Returns the local Zenoh ID.
 *
 * Unless the `session` is invalid, that ID is guaranteed to be non-zero.
 * In other words, this function returning an array of 16 zeros means you failed
 * to pass it a valid session.
 */
struct z_id_t z_info_zid(struct z_session_t session);
/**
 * Constructs a :c:type:`z_keyexpr_t` departing from a string.
 * It is a loaned key expression that aliases `name`.
 */
struct z_keyexpr_t z_keyexpr(const char *name);
/**
 * Returns the key expression's internal string by aliasing it.
 *
 * Currently exclusive to zenoh-c
 */
struct z_bytes_t z_keyexpr_as_bytes(struct z_keyexpr_t keyexpr);
/**
 * Canonizes the passed string in place, possibly shortening it by modifying `len`.
 *
 * Returns ``0`` upon success, negative values upon failure.
 * Returns a negative value if canonization failed, which indicates that the passed string was an invalid
 * key expression for reasons other than a non-canon form.
 *
 * May SEGFAULT if `start` is NULL or lies in read-only memory (as values initialized with string litterals do).
 */
int8_t z_keyexpr_canonize(char *start,
                          uintptr_t *len);
/**
 * Canonizes the passed string in place, possibly shortening it by placing a new null-terminator.
 *
 * Returns ``0`` upon success, negative values upon failure.
 * Returns a negative value if canonization failed, which indicates that the passed string was an invalid
 * key expression for reasons other than a non-canon form.
 *
 * May SEGFAULT if `start` is NULL or lies in read-only memory (as values initialized with string litterals do).
 */
int8_t z_keyexpr_canonize_null_terminated(char *start);
/**
 * Returns ``true`` if `keyexpr` is valid.
 */
bool z_keyexpr_check(const struct z_owned_keyexpr_t *keyexpr);
/**
 * Performs string concatenation and returns the result as a `z_owned_keyexpr_t`.
 * In case of error, the return value will be set to its invalidated state.
 *
 * You should probably prefer `z_keyexpr_join` as Zenoh may then take advantage of the hierachical separation it inserts.
 *
 * To avoid odd behaviors, concatenating a key expression starting with `*` to one ending with `*` is forbidden by this operation,
 * as this would extremely likely cause bugs.
 */
struct z_owned_keyexpr_t z_keyexpr_concat(struct z_keyexpr_t left,
                                          const char *right_start,
                                          uintptr_t right_len);
/**
 * Frees `keyexpr` and invalidates it for double-drop safety.
 */
void z_keyexpr_drop(struct z_owned_keyexpr_t *keyexpr);
/**
 * Returns ``0`` if both ``left`` and ``right`` are equal. Otherwise, it returns a ``-1``, or other ``negative value`` for errors.
 */
int8_t z_keyexpr_equals(struct z_keyexpr_t left,
                        struct z_keyexpr_t right);
/**
 * Returns ``0`` if ``left`` includes ``right``, i.e. the set defined by ``left`` contains every key belonging to the set
 * defined by ``right``. Otherwise, it returns a ``-1``, or other ``negative value`` for errors.
 */
int8_t z_keyexpr_includes(struct z_keyexpr_t left,
                          struct z_keyexpr_t right);
/**
 * Returns ``0`` if the keyexprs intersect, i.e. there exists at least one key which is contained in both of the
 * sets defined by ``left`` and ``right``. Otherwise, it returns a ``-1``, or other ``negative value`` for errors.
 */
int8_t z_keyexpr_intersects(struct z_keyexpr_t left,
                            struct z_keyexpr_t right);
/**
 * Returns ``0`` if the passed string is a valid (and canon) key expression.
 * Otherwise returns error value
 */
int8_t z_keyexpr_is_canon(const char *start, uintptr_t len);
/**
 * Returns ``true`` if `keyexpr` is initialized.
 */
bool z_keyexpr_is_initialized(const struct z_keyexpr_t *keyexpr);
/**
 * Performs path-joining (automatically inserting) and returns the result as a `z_owned_keyexpr_t`.
 * In case of error, the return value will be set to its invalidated state.
 */
struct z_owned_keyexpr_t z_keyexpr_join(struct z_keyexpr_t left, struct z_keyexpr_t right);
/**
 * Returns a :c:type:`z_keyexpr_t` loaned from :c:type:`z_owned_keyexpr_t`.
 */
struct z_keyexpr_t z_keyexpr_loan(const struct z_owned_keyexpr_t *keyexpr);
/**
 * Constructs a :c:type:`z_keyexpr_t` departing from a string, copying the passed string.
 */
struct z_owned_keyexpr_t z_keyexpr_new(const char *name);
/**
 * Constructs a null safe-to-drop value of 'z_owned_keyexpr_t' type
 */
struct z_owned_keyexpr_t z_keyexpr_null(void);
/**
 * Constructs a null-terminated string departing from a :c:type:`z_keyexpr_t`.
 * The user is responsible of droping the returned string using `z_drop`
 */
struct z_owned_str_t z_keyexpr_to_string(struct z_keyexpr_t keyexpr);
/**
 * Constructs a :c:type:`z_keyexpr_t` departing from a string without checking any of `z_keyexpr_t`'s assertions:
 *
 *  - `name` MUST be valid UTF8.
 *  - `name` MUST follow the Key Expression specification, ie:
 *
 *   - MUST NOT contain `//`, MUST NOT start nor end with `/`, MUST NOT contain any of the characters `?#$`.
 *   - any instance of `**` may only be lead or followed by `/`.
 *   - the key expression must have canon form.
 *
 * It is a loaned key expression that aliases `name`.
 */
struct z_keyexpr_t z_keyexpr_unchecked(const char *name);
/**
 * Opens a zenoh session. Should the session opening fail, `z_check` ing the returned value will return `false`.
 */
struct z_owned_session_t z_open(struct z_owned_config_t *config);
/**
 * Returns ``true`` if `pub` is valid.
 */
bool z_publisher_check(const struct z_owned_publisher_t *pbl);
/**
 * Sends a `DELETE` message onto the publisher's key expression.
 *
 * Returns:
 *     ``0`` in case of success, ``1`` in case of failure.
 */
int8_t z_publisher_delete(struct z_publisher_t publisher,
                          const struct z_publisher_delete_options_t *_options);
/**
 * Constructs the default values for the delete operation via a publisher entity.
 *
 * Returns:
 *   Returns the constructed :c:type:`z_publisher_delete_options_t`.
 */
struct z_publisher_delete_options_t z_publisher_delete_options_default(void);
/**
 * Returns the key expression of the publisher
 */
struct z_owned_keyexpr_t z_publisher_keyexpr(struct z_publisher_t publisher);
/**
 * Returns a :c:type:`z_publisher_t` loaned from `p`.
 */
struct z_publisher_t z_publisher_loan(const struct z_owned_publisher_t *p);
/**
 * Constructs a null safe-to-drop value of 'z_owned_publisher_t' type
 */
struct z_owned_publisher_t z_publisher_null(void);
/**
 * Constructs the default value for :c:type:`z_publisher_options_t`.
 */
struct z_publisher_options_t z_publisher_options_default(void);
/**
 * Sends a `PUT` message onto the publisher's key expression.
 *
 * The payload's encoding can be sepcified through the options.
 *
 * Parameters:
 *     session: The zenoh session.
 *     payload: The value to put.
 *     len: The length of the value to put.
 *     options: The publisher put options.
 * Returns:
 *     ``0`` in case of success, negative values in case of failure.
 */
int8_t z_publisher_put(struct z_publisher_t publisher,
                       const uint8_t *payload,
                       uintptr_t len,
                       const struct z_publisher_put_options_t *options);
/**
 * Constructs the default value for :c:type:`z_publisher_put_options_t`.
 */
struct z_publisher_put_options_t z_publisher_put_options_default(void);
/**
 * Returns ``true`` if `sub` is valid.
 */
bool z_pull_subscriber_check(const struct z_owned_pull_subscriber_t *sub);
/**
 * Returns ``true`` if `sub` is valid.
 */
struct z_pull_subscriber_t z_pull_subscriber_loan(const struct z_owned_pull_subscriber_t *sub);
/**
 * Constructs a null safe-to-drop value of 'z_owned_pull_subscriber_t' type
 */
struct z_owned_pull_subscriber_t z_pull_subscriber_null(void);
/**
 * Constructs the default value for :c:type:`z_pull_subscriber_options_t`.
 */
struct z_pull_subscriber_options_t z_pull_subscriber_options_default(void);
/**
 * Put data.
 *
 * The payload's encoding can be sepcified through the options.
 *
 * Parameters:
 *     session: The zenoh session.
 *     keyexpr: The key expression to put.
 *     payload: The value to put.
 *     len: The length of the value to put.
 *     options: The put options.
 * Returns:
 *     ``0`` in case of success, negative values in case of failure.
 */
int8_t z_put(struct z_session_t session,
             struct z_keyexpr_t keyexpr,
             const uint8_t *payload,
             size_t len,
             const struct z_put_options_t *opts);
/**
 * Constructs the default value for :c:type:`z_put_options_t`.
 */
struct z_put_options_t z_put_options_default(void);
/**
 * Automatic query consolidation strategy selection.
 *
 * A query consolidation strategy will automatically be selected depending the query selector.
 * If the selector contains time range properties, no consolidation is performed.
 * Otherwise the :c:func:`z_query_consolidation_latest` strategy is used.
 *
 * Returns:
 *   Returns the constructed :c:type:`z_query_consolidation_t`.
 */
struct z_query_consolidation_t z_query_consolidation_auto(void);
/**
 * Creates a default :c:type:`z_query_consolidation_t` (consolidation mode AUTO).
 */
struct z_query_consolidation_t z_query_consolidation_default(void);
/**
 * Latest value consolidation.
 */
struct z_query_consolidation_t z_query_consolidation_latest(void);
/**
 * Monotonic consolidation.
 */
struct z_query_consolidation_t z_query_consolidation_monotonic(void);
/**
 * Disable consolidation.
 */
struct z_query_consolidation_t z_query_consolidation_none(void);
/**
 * Get a query's key by aliasing it.
 */
struct z_keyexpr_t z_query_keyexpr(const struct z_query_t *query);
/**
 * Get a query's `value selector <https://github.com/eclipse-zenoh/roadmap/tree/main/rfcs/ALL/Selectors>`_ by aliasing it.
 */
struct z_bytes_t z_query_parameters(const struct z_query_t *query);
/**
 * Send a reply to a query.
 *
 * This function must be called inside of a Queryable callback passing the
 * query received as parameters of the callback function. This function can
 * be called multiple times to send multiple replies to a query. The reply
 * will be considered complete when the Queryable callback returns.
 *
 * Parameters:
 *     query: The query to reply to.
 *     key: The key of this reply.
 *     payload: The value of this reply.
 *     len: The length of the value of this reply.
 *     options: The options of this reply.
 */
int8_t z_query_reply(const struct z_query_t *query,
                     struct z_keyexpr_t key,
                     const uint8_t *payload,
                     uintptr_t len,
                     const struct z_query_reply_options_t *options);
/**
 * Constructs the default value for :c:type:`z_query_reply_options_t`.
 */
struct z_query_reply_options_t z_query_reply_options_default(void);
/**
 * Create a default :c:type:`z_query_target_t`.
 */
enum z_query_target_t z_query_target_default(void);
/**
 * Get a query's `payload value <https://github.com/eclipse-zenoh/roadmap/blob/main/rfcs/ALL/Query%20Payload.md>`_ by aliasing it.
 *
 * **WARNING: This API has been marked as unstable: it works as advertised, but it may change in a future release.**
 */
struct z_value_t z_query_value(const struct z_query_t *query);
/**
 * Returns ``true`` if `qable` is valid.
 */
bool z_queryable_check(const struct z_owned_queryable_t *qable);
/**
 * Constructs a null safe-to-drop value of 'z_owned_queryable_t' type
 */
struct z_owned_queryable_t z_queryable_null(void);
/**
 * Constructs the default value for :c:type:`z_query_reply_options_t`.
 */
struct z_queryable_options_t z_queryable_options_default(void);
/**
 * Calls the closure. Calling an uninitialized closure is a no-op.
 */
bool z_reply_channel_closure_call(const struct z_owned_reply_channel_closure_t *closure,
                                  struct z_owned_reply_t *sample);
/**
 * Drops the closure. Droping an uninitialized closure is a no-op.
 */
void z_reply_channel_closure_drop(struct z_owned_reply_channel_closure_t *closure);
/**
 * Constructs a null safe-to-drop value of 'z_owned_reply_channel_closure_t' type
 */
struct z_owned_reply_channel_closure_t z_reply_channel_closure_null(void);
void z_reply_channel_drop(struct z_owned_reply_channel_t *channel);
/**
 * Constructs a null safe-to-drop value of 'z_owned_reply_channel_t' type
 */
struct z_owned_reply_channel_t z_reply_channel_null(void);
/**
 * Returns ``true`` if `reply_data` is valid.
 */
bool z_reply_check(const struct z_owned_reply_t *reply_data);
/**
 * Frees `reply_data`, invalidating it for double-drop safety.
 */
void z_reply_drop(struct z_owned_reply_t *reply_data);
/**
 * Yields the contents of the reply by asserting it indicates a failure.
 *
 * You should always make sure that :c:func:`z_reply_is_ok` returns ``false`` before calling this function.
 */
struct z_value_t z_reply_err(const struct z_owned_reply_t *reply);
/**
 * Returns ``true`` if the queryable answered with an OK, which allows this value to be treated as a sample.
 *
 * If this returns ``false``, you should use :c:func:`z_check` before trying to use :c:func:`z_reply_err` if you want to process the error that may be here.
 */
bool z_reply_is_ok(const struct z_owned_reply_t *reply);
/**
 * Returns an invalidated :c:type:`z_owned_reply_t`.
 *
 * This is useful when you wish to take ownership of a value from a callback to :c:func:`z_get`:
 *
 *     - copy the value of the callback's argument's pointee,
 *     - overwrite the pointee with this function's return value,
 *     - you are now responsible for dropping your copy of the reply.
 */
struct z_owned_reply_t z_reply_null(void);
/**
 * Yields the contents of the reply by asserting it indicates a success.
 *
 * You should always make sure that :c:func:`z_reply_is_ok` returns ``true`` before calling this function.
 */
struct z_sample_t z_reply_ok(const struct z_owned_reply_t *reply);
/**
 * Scout for routers and/or peers.
 *
 * Parameters:
 *     what: A whatami bitmask of zenoh entities kind to scout for.
 *     config: A set of properties to configure the scouting.
 *     timeout: The time (in milliseconds) that should be spent scouting.
 *
 * Returns 0 if successful, negative values upon failure.
 */
int8_t z_scout(struct z_owned_scouting_config_t *config, struct z_owned_closure_hello_t *callback);
bool z_scouting_config_check(const struct z_owned_scouting_config_t *config);
struct z_owned_scouting_config_t z_scouting_config_default(void);
void z_scouting_config_drop(struct z_owned_scouting_config_t *config);
struct z_owned_scouting_config_t z_scouting_config_from(struct z_config_t config);
struct z_owned_scouting_config_t z_scouting_config_null(void);
/**
 * Returns ``true`` if `session` is valid.
 */
bool z_session_check(const struct z_owned_session_t *session);
/**
 * Returns a :c:type:`z_session_t` loaned from `s`.
 *
 * This handle doesn't increase the refcount of the session, but does allow to do so with `zc_session_rcinc`.
 *
 * # Safety
 * The returned `z_session_t` aliases `z_owned_session_t`'s internal allocation,
 * attempting to use it after all owned handles to the session (including publishers, queryables and subscribers)
 * have been destroyed is UB (likely SEGFAULT)
 */
struct z_session_t z_session_loan(const struct z_owned_session_t *s);
/**
 * Constructs a null safe-to-drop value of 'z_owned_session_t' type
 */
struct z_owned_session_t z_session_null(void);
/**
 * Returns ``true`` if `strs` is valid.
 */
bool z_str_array_check(const struct z_owned_str_array_t *strs);
/**
 * Frees `strs` and invalidates it for double-drop safety.
 */
void z_str_array_drop(struct z_owned_str_array_t *strs);
/**
 * Returns a :c:type:`z_str_array_t` loaned from :c:type:`z_owned_str_array_t`.
 */
struct z_str_array_t z_str_array_loan(const struct z_owned_str_array_t *strs);
/**
 * Returns ``true`` if `s` is a valid string
 */
bool z_str_check(const struct z_owned_str_t *s);
/**
 * Frees `z_owned_str_t`, invalidating it for double-drop safety.
 */
void z_str_drop(struct z_owned_str_t *s);
/**
 * Returns :c:type:`z_str_t` structure loaned from :c:type:`z_owned_str_t`.
 */
const char *z_str_loan(const struct z_owned_str_t *s);
/**
 * Returns undefined `z_owned_str_t`
 */
struct z_owned_str_t z_str_null(void);
/**
 * Returns ``true`` if `sub` is valid.
 */
bool z_subscriber_check(const struct z_owned_subscriber_t *sub);
/**
 * Returns the key expression of the subscriber.
 */
struct z_owned_keyexpr_t z_subscriber_keyexpr(struct z_subscriber_t subscriber);
/**
 * Returns a :c:type:`z_subscriber_t` loaned from `p`.
 */
struct z_subscriber_t z_subscriber_loan(const struct z_owned_subscriber_t *p);
/**
 * Constructs a null safe-to-drop value of 'z_owned_subscriber_t' type
 */
struct z_owned_subscriber_t z_subscriber_null(void);
/**
 * Constructs the default value for :c:type:`z_subscriber_options_t`.
 */
struct z_subscriber_options_t z_subscriber_options_default(void);
/**
 * Pull data for :c:type:`z_owned_pull_subscriber_t`. The pulled data will be provided
 * by calling the **callback** function provided to the :c:func:`z_declare_subscriber` function.
 *
 * Parameters:
 *     sub: The :c:type:`z_owned_pull_subscriber_t` to pull from.
 */
int8_t z_subscriber_pull(struct z_pull_subscriber_t sub);
/**
 * Returns ``true`` if `ts` is a valid timestamp
 */
bool z_timestamp_check(struct z_timestamp_t ts);
/**
 * Undeclare the key expression generated by a call to :c:func:`z_declare_keyexpr`.
 */
int8_t z_undeclare_keyexpr(struct z_session_t session, struct z_owned_keyexpr_t *kexpr);
/**
 * Undeclares the given :c:type:`z_owned_publisher_t`, droping it and invalidating it for double-drop safety.
 */
int8_t z_undeclare_publisher(struct z_owned_publisher_t *publisher);
/**
 * Undeclares the given :c:type:`z_owned_pull_subscriber_t`, droping it and invalidating it for double-drop safety.
 */
int8_t z_undeclare_pull_subscriber(struct z_owned_pull_subscriber_t *sub);
/**
 * Undeclares a `z_owned_queryable_t`, droping it and invalidating it for doube-drop safety.
 *
 * Parameters:
 *     qable: The :c:type:`z_owned_queryable_t` to undeclare.
 */
int8_t z_undeclare_queryable(struct z_owned_queryable_t *qable);
/**
 * Undeclares the given :c:type:`z_owned_subscriber_t`, droping it and invalidating it for double-drop safety.
 */
int8_t z_undeclare_subscriber(struct z_owned_subscriber_t *sub);
/**
 * Constructs a configuration by parsing a file at `path`. Currently supported format is JSON5, a superset of JSON.
 */
struct z_owned_config_t zc_config_from_file(const char *path);
/**
 * Reads a configuration from a JSON-serialized string, such as '{mode:"client",connect:{endpoints:["tcp/127.0.0.1:7447"]}}'.
 *
 * Passing a null-ptr will result in a gravestone value (`z_check(x) == false`).
 */
struct z_owned_config_t zc_config_from_str(const char *s);
/**
 * Gets the property with the given path key from the configuration, returning an owned, null-terminated, JSON serialized string.
 * Use `z_drop` to safely deallocate this string
 */
struct z_owned_str_t zc_config_get(struct z_config_t config,
                                   const char *key);
/**
 * Inserts a JSON-serialized `value` at the `key` position of the configuration.
 *
 * Returns 0 if successful, a negative value otherwise.
 */
int8_t zc_config_insert_json(struct z_config_t config, const char *key, const char *value);
/**
 * Converts `config` into a JSON-serialized string, such as '{"mode":"client","connect":{"endpoints":["tcp/127.0.0.1:7447"]}}'.
 */
struct z_owned_str_t zc_config_to_string(struct z_config_t config);
/**
 * Initialises the zenoh runtime logger.
 *
 * Note that unless you built zenoh-c with the `logger-autoinit` feature disabled,
 * this will be performed automatically by `z_open` and `z_scout`.
 */
void zc_init_logger(void);
/**
 * Constructs a :c:type:`z_keyexpr_t` departing from a string.
 * It is a loaned key expression that aliases `name`.
 */
struct z_keyexpr_t zc_keyexpr_from_slice(const char *name, uintptr_t len);
/**
 * Constructs a :c:type:`z_keyexpr_t` departing from a string without checking any of `z_keyexpr_t`'s assertions:
 * - `name` MUST be valid UTF8.
 * - `name` MUST follow the Key Expression specification, ie:
 *   - MUST NOT contain ``//``, MUST NOT start nor end with ``/``, MUST NOT contain any of the characters ``?#$``.
 *   - any instance of ``**`` may only be lead or followed by ``/``.
 *   - the key expression must have canon form.
 *
 * It is a loaned key expression that aliases `name`.
 */
struct z_keyexpr_t zc_keyexpr_from_slice_unchecked(const char *start,
                                                   uintptr_t len);
/**
 * Returns `false` if `payload` is the gravestone value.
 */
bool zc_payload_check(const struct zc_owned_payload_t *payload);
/**
 * Decrements `payload`'s backing refcount, releasing the memory if appropriate.
 */
void zc_payload_drop(struct zc_owned_payload_t *payload);
/**
 * Constructs `zc_owned_payload_t`'s gravestone value.
 */
struct zc_owned_payload_t zc_payload_null(void);
/**
 * Clones the `payload` by incrementing its reference counter.
 */
struct zc_owned_payload_t zc_payload_rcinc(const struct zc_owned_payload_t *payload);
/**
 * Sends a `PUT` message onto the publisher's key expression, transfering the buffer ownership.
 *
 * This is avoids copies when transfering data that was either:
 * - `zc_sample_payload_rcinc`'d from a sample, when forwarding samples from a subscriber/query to a publisher
 * - constructed from a `zc_owned_shmbuf_t`
 *
 * The payload's encoding can be sepcified through the options.
 *
 * Parameters:
 *     session: The zenoh session.
 *     payload: The value to put.
 *     len: The length of the value to put.
 *     options: The publisher put options.
 * Returns:
 *     ``0`` in case of success, negative values in case of failure.
 */
int8_t zc_publisher_put_owned(struct z_publisher_t publisher,
                              struct zc_owned_payload_t *payload,
                              const struct z_publisher_put_options_t *options);
/**
 * Put data, transfering the buffer ownership.
 *
 * This is avoids copies when transfering data that was either:
 * - `zc_sample_payload_rcinc`'d from a sample, when forwarding samples from a subscriber/query to a publisher
 * - constructed from a `zc_owned_shmbuf_t`
 *
 * The payload's encoding can be sepcified through the options.
 *
 * Parameters:
 *     session: The zenoh session.
 *     keyexpr: The key expression to put.
 *     payload: The value to put.
 *     options: The put options.
 * Returns:
 *     ``0`` in case of success, negative values in case of failure.
 */
int8_t zc_put_owned(struct z_session_t session,
                    struct z_keyexpr_t keyexpr,
                    struct zc_owned_payload_t *payload,
                    const struct z_put_options_t *opts);
/**
 * Creates a new blocking fifo channel, returned as a pair of closures.
 *
 * If `bound` is different from 0, that channel will be bound and apply back-pressure when full.
 *
 * The `send` end should be passed as callback to a `z_get` call.
 *
 * The `recv` end is a synchronous closure that will block until either a `z_owned_reply_t` is available,
 * which it will then return; or until the `send` closure is dropped and all replies have been consumed,
 * at which point it will return an invalidated `z_owned_reply_t`, and so will further calls.
 */
struct z_owned_reply_channel_t zc_reply_fifo_new(uintptr_t bound);
/**
 * Creates a new non-blocking fifo channel, returned as a pair of closures.
 *
 * If `bound` is different from 0, that channel will be bound and apply back-pressure when full.
 *
 * The `send` end should be passed as callback to a `z_get` call.
 *
 * The `recv` end is a synchronous closure that will block until either a `z_owned_reply_t` is available,
 * which it will then return; or until the `send` closure is dropped and all replies have been consumed,
 * at which point it will return an invalidated `z_owned_reply_t`, and so will further calls.
 */
struct z_owned_reply_channel_t zc_reply_non_blocking_fifo_new(uintptr_t bound);
/**
 * Clones the sample's payload by incrementing its backing refcount (this doesn't imply any copies).
 */
struct zc_owned_payload_t zc_sample_payload_rcinc(const struct z_sample_t *sample);
/**
 * Increments the session's reference count, returning a new owning handle.
 */
struct z_owned_session_t zc_session_rcinc(struct z_session_t session);
/**
 * Allocates a buffer of size `capacity` in the manager's memory.
 *
 * # Safety
 * Calling this function concurrently with other shm functions on the same manager is UB.
 */
struct zc_owned_shmbuf_t zc_shm_alloc(const struct zc_owned_shm_manager_t *manager,
                                      uintptr_t capacity);
/**
 * Runs a defragmentation pass on the SHM manager.
 *
 * Note that this doesn't trigger a garbage collection pass, nor does it move currently allocated data.
 *
 * # Safety
 * Calling this function concurrently with other shm functions on the same manager is UB.
 */
uintptr_t zc_shm_defrag(const struct zc_owned_shm_manager_t *manager);
/**
 * Runs a garbage collection pass on the SHM manager.
 *
 * Returns the number of bytes that have been freed by the pass.
 *
 * # Safety
 * Calling this function concurrently with other shm functions on the same manager is UB.
 */
uintptr_t zc_shm_gc(const struct zc_owned_shm_manager_t *manager);
bool zc_shm_manager_check(const struct zc_owned_shm_manager_t *manager);
void zc_shm_manager_drop(struct zc_owned_shm_manager_t *manager);
struct zc_owned_shm_manager_t zc_shm_manager_new(struct z_session_t session,
                                                 const char *id,
                                                 uintptr_t size);
struct zc_owned_shm_manager_t zc_shm_manager_null(void);
/**
 * Returns the capacity of the SHM buffer.
 */
uintptr_t zc_shmbuf_capacity(const struct zc_owned_shmbuf_t *buf);
/**
 * Returns `false` if `buf` is in its gravestone state.
 */
bool zc_shmbuf_check(const struct zc_owned_shmbuf_t *buf);
/**
 * Drops the SHM buffer, decrementing its backing reference counter.
 */
void zc_shmbuf_drop(struct zc_owned_shmbuf_t *buf);
/**
 * Constructs an owned payload from an owned SHM buffer.
 */
struct zc_owned_payload_t zc_shmbuf_into_payload(struct zc_owned_shmbuf_t *buf);
/**
 * Returns the length of the SHM buffer.
 *
 * Note that when constructing an SHM buffer, length is defaulted to its capacity.
 */
uintptr_t zc_shmbuf_length(const struct zc_owned_shmbuf_t *buf);
/**
 * Constructs a null safe-to-drop value of type `zc_owned_shmbuf_t`
 */
struct zc_owned_shmbuf_t zc_shmbuf_null(void);
/**
 * Returns the start of the SHM buffer.
 */
uint8_t *zc_shmbuf_ptr(const struct zc_owned_shmbuf_t *buf);
/**
 * Sets the length of the SHM buffer.
 *
 * This lets Zenoh know how much of the data to write over the network when sending the value to non-SHM-compatible neighboors.
 */
void zc_shmbuf_set_length(const struct zc_owned_shmbuf_t *buf,
                          uintptr_t len);
