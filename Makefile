all: mujoco

MUJOCO = mujoco-2.1.5-linux-x86_64.tar.gz
TARBALL = build/$(MUJOCO)
TARBALL_URL = https://github.com/deepmind/mujoco/releases/download/2.1.5/$(MUJOCO)
SOURCE_DIR = build/mujoco-2.1.5
SHA256SUM = 3f1804d28833295a310aac23279401936f2558dee63cd3778429577e4ab55dff
SHA256SUM_ACTUAL = sha256sum $(TARBALL) | cut -d ' ' -f 1
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

mujoco: $(SOURCE_DIR)/unpacked
	echo "$(SHA256SUM)  build/$(MUJOCO)" | sha256sum --check

clean:
	-rm -rf $(SOURCE_DIR)

wipe: clean
	-rm -rf build