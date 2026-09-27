FROM python:3.12-slim

WORKDIR /app

# 依存関係を先にインストールしてビルドキャッシュを効かせる
COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt

# backend パッケージとして配置（launcher.py からの import と揃える）
COPY . backend/

EXPOSE 8100
CMD ["uvicorn", "backend.main:app", "--host", "0.0.0.0", "--port", "8100"]
