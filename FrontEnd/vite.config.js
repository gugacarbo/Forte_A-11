import { defineConfig } from 'vite'
import preact from '@preact/preset-vite'
import svgr from "vite-plugin-svgr";

// https://vitejs.dev/config/
export default defineConfig({
  build: {
    rollupOptions: {
      output: {
        chunkFileNames: () => {
          return `[name].js`;
        },
        entryFileNames: () => {
          return `[name].js`;
        },
        assetFileNames: ({ name }) => {
          if (/\.svg$/i.test(name)) {
            return `svg/[name].[ext]`;
          }
          if (/\.(jpg|jpeg|png|gif)$/i.test(name)) {
            return `images/[name].[ext]`;
          }
          return `[name].[ext]`;
        },
      },
    },
  },
  plugins: [preact(), svgr()],
})
