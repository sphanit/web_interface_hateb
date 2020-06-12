# ifneq ($(wildcard /local/users/username),)
#   HOME = /local/users/username
# endif

all: stop prepare build run

.PHONY: prepare build build-image run stop

SPECTRE_URL = https://github.com/picturepan2/spectre/raw/58e8e7e600c8fb0e2c292e8a74818a5a1b587c64/dist/spectre.min.css

prepare:
	@docker run --rm -it -v $(CURDIR):/workspace harmish/typescript typings install --save
	@# @docker run --rm -it -v $(HOME)/ros/web:$(HOME)/ros/web -w $(PWD) harmish/typescript typings install --save
	@docker run --rm -it -v $(CURDIR):/workspace busybox test -e /workspace/dist/spectre.min.css || wget $(SPECTRE_URL) -P dist
	@# @cp $(HOME)/ros/web/rosjs-typings/rosmsgs/build/rosmsgs.js $(CURDIR)/dist/
	@# @cp $(HOME)/ros/web/ros3dnavts/build/ros3dnav.js $(CURDIR)/dist/

build:
	@docker run --rm -v $(CURDIR):/workspace harmish/typescript tsc -t ES6 -p /workspace || true

build-image:
	@docker build -t harmish/hateb_viz .

run: build-image
	@docker run -d --name=hateb_viz \
     --restart=unless-stopped \
     -p 81:80 \
     harmish/hateb_viz

stop::
	@docker stop hateb_viz >/dev/null 2>&1 || true
	@docker rm hateb_viz >/dev/null 2>&1 || true
