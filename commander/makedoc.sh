echo "Generating Docs..."
doxygen .Doxyfile
cd ./doc/latex/
make || echo "Errors in making"

echo "Done!"
