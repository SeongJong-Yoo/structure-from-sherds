cd $SRC_DIR/build
make install
rm $PREFIX/lib/libflang${SHLIB_EXT}
rm $PREFIX/lib/libflangrti${SHLIB_EXT}
rm $PREFIX/lib/libompstub${SHLIB_EXT}

for CHANGE in "activate" "deactivate"
do
    mkdir -p "${PREFIX}/etc/conda/${CHANGE}.d"
    cp "${RECIPE_DIR}/${CHANGE}.sh" "${PREFIX}/etc/conda/${CHANGE}.d/${PKG_NAME}_${CHANGE}.sh"
done
