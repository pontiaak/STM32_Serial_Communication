/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7 */

#ifndef PB_GENERATOR_BIN_PROTO_FILES_TEST_PB_H_INCLUDED
#define PB_GENERATOR_BIN_PROTO_FILES_TEST_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _TestMessage {
    int32_t test_number;
} TestMessage;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define TestMessage_init_default                 {0}
#define TestMessage_init_zero                    {0}

/* Field tags (for use in manual encoding/decoding) */
#define TestMessage_test_number_tag              1

/* Struct field encoding specification for nanopb */
#define TestMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, INT32,    test_number,       1)
#define TestMessage_CALLBACK NULL
#define TestMessage_DEFAULT NULL

extern const pb_msgdesc_t TestMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define TestMessage_fields &TestMessage_msg

/* Maximum encoded size of messages (where known) */
#define TestMessage_size                         11

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
