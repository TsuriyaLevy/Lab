
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml
cd ..
mv ./ORB_SLAM2/pointData.csv ./opencv/modules/python/python3/pointData.csv
cd ./opencv/modules/python/python3
chmod +x find_exit.py
python3 ./find_exit.py

