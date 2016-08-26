#!/bin/bash

gunicorn -k eventlet -w 4 -b :8080 config:application
