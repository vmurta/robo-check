# Important considerations when running rai

1. If opening or accessing a file, you MUST prefix the path with `/src/` so that rai can see the file.
2. You must add every build command you use to `rai_build.yml` manually, keeping in mind that all your files will be in `/src` directory.
3. Printing in rai works the same as usual.