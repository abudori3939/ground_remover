# ground_remover
PointCloudで地面を除去するサンプルコードです。

## 概要
このプログラムは、PCD (Point Cloud Data) または PLY (Polygon File Format) ファイル形式の点群データを入力として受け取り、PCL (Point Cloud Library) の ProgressiveMorphologicalFilter を使用して地面を除去します。処理後の点群は `filtered_cloud.pcd` という名前で保存され、処理前後の点群がPCLViewerで表示されます。

## 依存関係
- PCL (Point Cloud Library)
- CMake (ビルドシステム)

## ビルド方法
1. 依存関係をインストールします:
   ```bash
   sh install.sh
   ```
2. ビルドディレクトリを作成し、移動します:
   ```bash
   mkdir build && cd build
   ```
3. CMakeを実行してビルドファイルを作成します:
   ```bash
   cmake ..
   ```
4. プログラムをビルドします:
   ```bash
   make
   ```

## 使用方法
ビルド後、`build` ディレクトリからプログラムを実行できます:
```bash
./ground_remover <入力ファイルパス>
```
例:
```bash
./ground_remover ../sample_cloud.pcd
```
`<入力ファイルパス>` には、処理したいPCDまたはPLYファイルのパスを指定してください。
