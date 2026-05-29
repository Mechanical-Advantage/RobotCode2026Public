pip install pybind11
python setup.py build_ext --inplace
cd ..
pip install -e ./aruco_max_cpp --no-build-isolation