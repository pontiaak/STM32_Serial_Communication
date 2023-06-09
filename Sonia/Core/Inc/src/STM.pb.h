/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7 */

#ifndef PB_GENERATOR_BIN_PROTO_FILES_STM_PB_H_INCLUDED
#define PB_GENERATOR_BIN_PROTO_FILES_STM_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _STMMessage {
    int32_t messageId;
    int32_t temperatureInt;
    int32_t temperatureFraction;
    bool has_fakeMeasure;
    int32_t fakeMeasure;
} STMMessage;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define STMMessage_init_default                  {0, 0, 0, false, 0}
#define STMMessage_init_zero                     {0, 0, 0, false, 0}

/* Field tags (for use in manual encoding/decoding) */
#define STMMessage_messageId_tag                 1
#define STMMessage_temperatureInt_tag            2
#define STMMessage_temperatureFraction_tag       3
#define STMMessage_fakeMeasure_tag               4

/* Struct field encoding specification for nanopb */
#define STMMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, INT32,    messageId,         1) \
X(a, STATIC,   REQUIRED, INT32,    temperatureInt,    2) \
X(a, STATIC,   REQUIRED, INT32,    temperatureFraction,   3) \
X(a, STATIC,   OPTIONAL, INT32,    fakeMeasure,       4)
#define STMMessage_CALLBACK NULL
#define STMMessage_DEFAULT NULL

extern const pb_msgdesc_t STMMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define STMMessage_fields &STMMessage_msg

/* Maximum encoded size of messages (where known) */
#define STMMessage_size                          44

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
