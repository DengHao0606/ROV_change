echo "开始清理编译记录"
rm -rf build
rm -rf install
rm -rf log
echo "清理完成"

echo "开始重新编译"
colcon build
echo "重新编译完成"

echo "安装编译结果"
source install/setup.bash