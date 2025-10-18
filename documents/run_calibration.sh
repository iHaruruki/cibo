#!/bin/bash

# カメラキャリブレーション実行スクリプト
# 8x6チェスボードを使った2カメラ間キャリブレーション

echo "================================================"
echo "Camera Calibration Script - Cibo Package"
echo "8x6 Chessboard Calibration for Two Cameras"
echo "================================================"
echo ""

# 環境設定
echo "環境設定中..."
cd ~/ros2_ws
source install/setup.bash

echo "使用方法："
echo "1. 基本的なキャリブレーション:"
echo "   ./run_calibration.sh basic"
echo ""
echo "2. テスト用画像でキャリブレーション:"
echo "   ./run_calibration.sh test"
echo ""
echo "3. RVizありでキャリブレーション:"
echo "   ./run_calibration.sh rviz"
echo ""

case "$1" in
    "basic")
        echo "基本キャリブレーションを開始..."
        echo ""
        echo "操作方法："
        echo "- スペースキー: チェスボード検出結果を保存"
        echo "- Cキー: キャリブレーション実行"
        echo "- Rキー: データリセット"
        echo "- Sキー: 結果保存"
        echo "- Lキー: 保存済み結果読み込み"
        echo "- Qキー: 終了"
        echo ""
        ros2 run cibo camera_calibration_node
        ;;
    "test")
        echo "テスト用画像でキャリブレーションを開始..."
        echo "テスト画像配信ノードを起動中..."
        
        # テスト画像配信ノードをバックグラウンドで起動
        gnome-terminal -- bash -c "
            source ~/ros2_ws/install/setup.bash;
            echo 'テスト画像配信中...';
            ros2 run cibo calibration_tester;
            exec bash"
        
        sleep 2
        
        echo "キャリブレーションノードを起動..."
        ros2 run cibo camera_calibration_node
        ;;
    "rviz")
        echo "RVizありでキャリブレーションを開始..."
        ros2 launch cibo camera_calibration_launch.py use_rviz:=true
        ;;
    "launch")
        echo "Launchファイルでキャリブレーションを開始..."
        ros2 launch cibo camera_calibration_launch.py
        ;;
    *)
        echo "引数を指定してください："
        echo "  basic  - 基本的なキャリブレーション"
        echo "  test   - テスト用画像でキャリブレーション"
        echo "  rviz   - RVizありでキャリブレーション"
        echo "  launch - Launchファイルを使用"
        exit 1
        ;;
esac
