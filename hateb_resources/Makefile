all: stop build run

.PHONY: build run stop

build:
	@cp -R $(CURDIR)/data $(CURDIR)/.data_copy && chmod -R a+rx $(CURDIR)/.data_copy
	@docker build -t harmish/hateb_resources .
	@rm -rf $(CURDIR)/.data_copy

run:
	@docker run -d --name=hateb_resources \
     --restart=unless-stopped \
     -p 80:80 \
     harmish/hateb_resources

stop::
	@docker stop hateb_resources >/dev/null 2>&1 || true
	@docker rm hateb_resources >/dev/null 2>&1 || true
