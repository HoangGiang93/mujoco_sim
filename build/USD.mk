all: usd

VERSION = 23.02
USD = USD-$(VERSION).tar.gz
TARBALL = $(USD)
TARBALL_URL = https://github.com/PixarAnimationStudios/USD/archive/refs/tags/v$(VERSION).tar.gz
SOURCE_DIR = USD-$(VERSION)
SHA256SUM = a8eefff722db0964ce5b11b90bcdc957f3569f1cf1d44c46026ecd229ce7535d
SHA256SUM_ACTUAL = sha256sum $(TARBALL) | cut -d ' ' -f 1
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

usd: $(SOURCE_DIR)/unpacked
	echo "$(SHA256SUM) $(USD)" | sha256sum --check

clean:
	-rm -rf $(SOURCE_DIR)

wipe: clean
	-rm -rf build