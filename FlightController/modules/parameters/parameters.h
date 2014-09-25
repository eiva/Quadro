/* Basic parameter structure */
struct param_s {
  uint8_t type;
  char * name;
  void * address;
};

#define PARAM_BYTES_MASK 0x03
#define PARAM_4BYTES 0x01

#define PARAM_TYPE_INT   (0x00<<2)
#define PARAM_TYPE_FLOAT (0x01<<2)

#define PARAM_SIGNED (0x00<<3)
#define PARAM_UNSIGNED (0x01<<3)

#define PARAM_VARIABLE (0x00<<7)
#define PARAM_GROUP    (0x01<<7)

#define PARAM_RONLY (1<<6)

#define PARAM_START 1
#define PARAM_STOP  0

#define PARAM_SYNC 0x02

// User-friendly macros
#define PARAM_UINT32 (PARAM_4BYTES | PARAM_TYPE_INT | PARAM_UNSIGNED)
#define PARAM_INT32  (PARAM_4BYTES | PARAM_TYPE_INT | PARAM_SIGNED)
#define PARAM_FLOAT  (PARAM_4BYTES | PARAM_TYPE_FLOAT | PARAM_SIGNED)


/* Macros */
#define PARAM_ADD(TYPE, NAME, ADDRESS) \
   { .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },

#define PARAM_ADD_GROUP(TYPE, NAME, ADDRESS) \
   { \
  .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },

#define PARAM_GROUP_START(NAME)  \
  static const struct param_s __params_##NAME[] __attribute__((section(".param." #NAME), used)) = { \
  PARAM_ADD_GROUP(PARAM_GROUP | PARAM_START, NAME, 0x0)

//#define PARAM_GROUP_START_SYNC(NAME, LOCK) PARAM_ADD_GROUP(PARAM_GROUP | PARAM_START, NAME, LOCK);

#define PARAM_GROUP_STOP(NAME) \
  PARAM_ADD_GROUP(PARAM_GROUP | PARAM_STOP, stop_##NAME, 0x0) \
  };

void InitParams();
uint16_t GetParamsCount();
bool GetParam(param_s& paramDesk, uint16_t paramIndex);
int16_t GetParam(param_s& paramDesk, const char* paramName);
