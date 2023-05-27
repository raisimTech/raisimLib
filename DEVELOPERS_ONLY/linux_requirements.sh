for PY_VERSION in 3.7 3.8 3.9 3.10
do
  sudo apt-get -y install python$PY_VERSION libpython$PY_VERSION-dev python$PY_VERSION-distutils
done