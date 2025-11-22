# trajectory_generator_2021
任意の点を指定してそれを補間する経路を生成するアプリケーションです．<br>
Python3上で動作します．<br>
経路データはCSV形式でエクスポートされます．<br>



# 必要なパッケージのインストール
pip，conda，aptなどお使いの環境に合わせてインストールしてください．
```
pip install pyqt5
pip install numpy
pip install matplotlib
```

一部環境ではpyqt5がうまく入らないので，その時は
```
sudo apt-get install python3-pyqt5
```
を試してみてください．

# 起動
ディレクトリ内に入ってPythonコマンドより起動．
```
cd trajectory_generator
python3 trajectory_generator_gui.py
```
