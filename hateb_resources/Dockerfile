FROM httpd:2.4-alpine

COPY ./httpd.conf /httpd_user.conf
COPY ./.data_copy /usr/local/apache2/htdocs/

RUN echo "Include /httpd_user.conf" >> /usr/local/apache2/conf/httpd.conf && \
    chmod -R a+rx /usr/local/apache2/htdocs

EXPOSE 80
