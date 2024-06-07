import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [react()],
  resolve: {
    alias: [{ find: "@", replacement: "/src" }],
  },
  build: {
    outDir: "../../ros2web_app/ros2web_app/ros2web_app/data/public",
    rollupOptions: {
      output: {
        chunkFileNames: "plugin/app/js/[name]-[hash].js",
        entryFileNames: "plugin/app/js/[name]-[hash].js",
        assetFileNames: "plugin/app/[name]-[hash][extname]",
      },
    },
  },
  server: {
    proxy: {
      "/plugin": {
        target: "http://0.0.0.0:8080",
      },
      "^/[a-zA-Z0-9_]+/app": {
        target: "http://0.0.0.0:8080",
      },
      "/ws": {
        target: "ws://0.0.0.0:8080",
        ws: true,
      },
    },
  },
});
