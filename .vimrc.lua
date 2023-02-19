vim.opt.autochdir = true
require('lspconfig')['clangd'].setup {
	cmd = { "/home/listout/Documents/w/esp-clang/bin/clangd" },
	on_attach = require("plugins.lsp-config").on_attach,
	cpabilities = require("plugins.lsp-config").cpabilities,
	lsp_flags = require("plugins.lsp-config").lsp_flags,
}
require('lspconfig')['cmake'].setup {
	cmd = { "/home/listout/.local/bin/cmake-language-server" },
	on_attach = require("plugins.lsp-config").on_attach,
	cpabilities = require("plugins.lsp-config").cpabilities,
	lsp_flags = require("plugins.lsp-config").lsp_flags,
}
