FROM python:3.12-slim

WORKDIR /app

# 依存関係を先にインストールしてビルドキャッシュを効かせる
COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt

# frontend パッケージとして配置（launcher.py からの import と揃える）
COPY . frontend/

EXPOSE 8180
CMD ["python", "-m", "frontend.main"]
