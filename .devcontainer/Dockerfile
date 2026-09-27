# 開発用: backend と frontend の依存関係をまとめてインストールする
FROM python:3.12-slim

COPY backend/requirements.txt /tmp/backend-requirements.txt
COPY frontend/requirements.txt /tmp/frontend-requirements.txt
RUN pip install --no-cache-dir \
    -r /tmp/backend-requirements.txt -r /tmp/frontend-requirements.txt
