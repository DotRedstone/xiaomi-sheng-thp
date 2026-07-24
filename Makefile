CXX ?= g++
CPPFLAGS += -Isrc -I/usr/include/libssc \
	$(shell pkg-config --cflags glib-2.0 gobject-2.0 gio-2.0)
CXXFLAGS ?= -O3 -flto -std=c++20
CXXFLAGS += -Wall -Wextra -Werror
LDLIBS += $(shell pkg-config --libs glib-2.0 gobject-2.0 gio-2.0) -lssc -lm

BUILD := build
TARGET := $(BUILD)/xiaomi-sheng-thp
SOURCES := \
	src/nvt_touch_core.cpp \
	src/nvt_finger_filter.cpp \
	src/nvt_stylus.cpp \
	src/nvt_p81c_control.cpp \
	src/nvt_pencil_posture.cpp \
	src/xiaomi_sheng_thp.cpp
HEADERS := \
	src/nvt_touch_core.hpp \
	src/nvt_finger_filter.hpp \
	src/nvt_stylus.hpp \
	src/nvt_focus_pen_pressure.hpp \
	src/nvt_p81c_control.hpp \
	src/nvt_pencil_posture.hpp

PREFIX ?= /usr
LIBEXECDIR ?= $(PREFIX)/libexec/xiaomi-sheng-thp
SYSTEMD_UNIT_DIR ?= $(PREFIX)/lib/systemd/system
DOCDIR ?= $(PREFIX)/share/doc/xiaomi-sheng-thp

.PHONY: all clean install

all: $(TARGET)

$(BUILD):
	mkdir -p $@

$(TARGET): $(SOURCES) $(HEADERS) | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SOURCES) -o $@ $(LDLIBS)

install: $(TARGET)
	install -Dm755 -s $(TARGET) \
		$(DESTDIR)$(LIBEXECDIR)/xiaomi-sheng-thp
	install -Dm644 systemd/xiaomi-sheng-thp.service \
		$(DESTDIR)$(SYSTEMD_UNIT_DIR)/xiaomi-sheng-thp.service
	install -Dm644 README.md \
		$(DESTDIR)$(DOCDIR)/README.md
	install -Dm644 LICENSE \
		$(DESTDIR)$(DOCDIR)/copyright

clean:
	rm -rf $(BUILD)
