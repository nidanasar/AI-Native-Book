const path = require('path');

module.exports = function (context, options) {
  return {
    name: 'docusaurus-plugin-floating-rag-ui',

    getClientModules() {
      return [path.resolve(__dirname, './client/floating-rag-ui.js')];
    },
  };
};