-- Project-specific neotest-gtest configuration.
-- This file tells the adapter how to build and locate test binaries.

return {
    -- How to build tests before running them
    -- This runs just build, which triggers cmake --build
    build_cmd = "cd " .. vim.fn.getcwd() .. " && cmake --build --preset debug",

    -- Map: test source file → test binary path
    -- neotest-gtest calls this to find the executable for a given test file
    get_test_binary = function(file_path)
        -- tests/test_cloud_mapper.cpp → build/test_cloud_mapper
        local filename = vim.fn.fnamemodify(file_path, ":t:r") -- "test_cloud_mapper"
        return vim.fn.getcwd() .. "/build/" .. filename
    end,
}
