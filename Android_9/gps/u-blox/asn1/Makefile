ASN_CONVERTER_SOURCES+=converter-sample.c

# This file may be used as an input for make(3)
# Remove the lines below to convert it into a pure .am file
CONVERTION_TOOL = convert
TARGET = libSupl.o
TARGET_LIB = libSupl.a
#CFLAGS += -I. -g -DPDU=ULP_PDU -DEMIT_ASN_DEBUG=1
#CFLAGS += -I. -g -DPDU=ULP_PDU
CFLAGS += -I. -g -fPIC
ASN_MODULE_SOURCES:= $(wildcard *.c)
OBJS=${ASN_MODULE_SOURCES:.c=.o} 

all: $(TARGET_LIB) 


conv: $(CONVERTION_TOOL)

$(CONVERTION_TOOL): $(TARGET_LIB)
	$(CC) $(CFLAGS) converter_sample/${ASN_CONVERTER_SOURCES} $(TARGET_LIB) -o converter_sample/$(CONVERTION_TOOL)

$(TARGET_LIB): ${OBJS}
	$(AR) rcs $(TARGET_LIB) ${OBJS} 

.SUFFIXES:
.SUFFIXES: .c .o

.c.o:
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(TARGET_LIB)
	rm -f $(TARGET)
	rm -f $(OBJS)
	rm -f converter_sample/$(CONVERTION_TOOL)

regen: regenerate-from-asn1-source

regenerate-from-asn1-source:
	asn1c -gen-PER -fnative-types -fskeletons-copy supl.asn rrlp.asn rrlp-components.asn
#	asn1c -gen-PER -fnative-types -fskeletons-copy supl.asn rrlp.asn rrlp-components.asn MAP-ExtensionDataTypes.asn MAP-LCS-DataTypes.asn

