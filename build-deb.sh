#!/bin/sh
set -eu

APP=xiaomi-sheng-thp
VERSION=0.3.1
ARCH="$(dpkg --print-architecture)"
ROOT="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
PKGROOT="$(mktemp -d)"
OUT="${ROOT}/${APP}_${VERSION}_${ARCH}.deb"

cleanup() {
	rm -rf "${PKGROOT}"
}
trap cleanup EXIT

chmod 755 "${PKGROOT}"
make -C "${ROOT}" clean all
make -C "${ROOT}" install DESTDIR="${PKGROOT}"

mkdir -p "${PKGROOT}/DEBIAN"
cat > "${PKGROOT}/DEBIAN/control" <<EOF
Package: ${APP}
Version: ${VERSION}
Section: utils
Priority: optional
Architecture: ${ARCH}
Depends: bluez, libc6, libstdc++6, systemd
Recommends: xiaomi-pen-status
Maintainer: siergtc <i@4t.pw>
Description: NT36532E userspace touch processor for Xiaomi Sheng
 Processes raw NT36532E THP frames and reports multitouch and Xiaomi Focus
 Pen input through uinput. A compatible kernel raw-stream ABI is required.
EOF

cat > "${PKGROOT}/DEBIAN/postinst" <<'EOF'
#!/bin/sh
set -e
if command -v systemctl >/dev/null 2>&1; then
	systemctl daemon-reload || true
	systemctl enable --now xiaomi-sheng-thp.service || true
fi
EOF
chmod 755 "${PKGROOT}/DEBIAN/postinst"

cat > "${PKGROOT}/DEBIAN/postrm" <<'EOF'
#!/bin/sh
set -e
if command -v systemctl >/dev/null 2>&1; then
	systemctl daemon-reload || true
fi
EOF
chmod 755 "${PKGROOT}/DEBIAN/postrm"

dpkg-deb --build --root-owner-group "${PKGROOT}" "${OUT}"
printf '%s\n' "${OUT}"
