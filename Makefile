all: mujoco

MUJOCO = mujoco-2.2.0-linux-x86_64.tar.gz
TARBALL = build/$(MUJOCO)
TARBALL_URL = https://github.com/deepmind/mujoco/releases/download/2.2.0/$(MUJOCO)
SOURCE_DIR = build/mujoco-2.2.0
SHA256SUM = 6313f24218c3ee832c73108e06a050341807166a44160fe3a322c441ec31bd2b
SHA256SUM_ACTUAL = sha256sum $(TARBALL) | cut -d ' ' -f 1
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

mujoco: $(SOURCE_DIR)/unpacked
	echo "$(SHA256SUM)  build/$(MUJOCO)" | sha256sum --check

clean:
	-rm -rf $(SOURCE_DIR)

wipe: clean
	-rm -rf build