# 03-dashboard/Dockerfile
FROM node:20-bullseye

WORKDIR /app

# Instala dependencias básicas de desarrollo
COPY src/package*.json ./
RUN npm install

COPY src/ .

EXPOSE 80

CMD ["npm", "start"]
