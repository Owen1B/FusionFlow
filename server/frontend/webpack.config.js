module.exports = {
  devServer: {
    allowedHosts: 'all',
    host: 'localhost',
    port: 3000,
    proxy: {
      '/api': {
        target: 'http://localhost:5000',
        changeOrigin: true
      }
    }
  }
}; 