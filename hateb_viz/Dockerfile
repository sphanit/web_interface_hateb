FROM httpd:2.4-alpine

COPY ./dist/* /usr/local/apache2/htdocs/dist/
COPY ./css/* /usr/local/apache2/htdocs/css/
COPY ./build/hatebviz.js /usr/local/apache2/htdocs/
COPY ./index.html /usr/local/apache2/htdocs/
COPY ./test.html /usr/local/apache2/htdocs/
COPY ./experiments.json /usr/local/apache2/htdocs/

RUN chmod -R a+rx /usr/local/apache2/htdocs

EXPOSE 80
