all: mujoco

MUJOCO = mujoco-2.1.5-linux-x86_64.tar.gz
TARBALL = build/$(MUJOCO)
TARBALL_URL = https://github.com/deepmind/mujoco/releases/download/2.1.5/$(MUJOCO)
SOURCE_DIR = build/mujoco-2.1.5
SHA256_FILE = $(MUJOCO).sha256
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

mujoco: $(SOURCE_DIR)/unpacked
	-rm -f build/$(MUJOCO)

clean:
	-rm -rf $(SOURCE_DIR)

wipe: clean
	-rm -rf build