# FM-7 Emulator for Raspberry Pi Pico
# WORK IN PROGRESS
(作成中のためここに書いてあっても未実装の機能があります)

# 概要

Raspberry Pi Pico で動作する、
FM-7 のエミュレータになります。
BML3 エミュレータの派生品です。

以下の機能を実装しています。

- メインシステム
- メインメモリ 64KB
- PSG
- Beep 出力
- テープ出力
- サブシステム
- VRAM 48KB
- 作業用メモリ

- 漢字 ROM (オプション)

Pico を 225MHz で動かしていますが、ほぼ実機の速度が出ていると思います。

# 注意

いつものように FM-7 実機の ROM が必要です。
`fm7rom_dummy.h` を `fm7rom.h` にリネームの上以下のデータを入れてください。

- F-BASIC
- Boot ROM (BASIC/DOS)
- サブシステム ROM

漢字ROM が必要な場合は、同様に漢字ROM の内容(128KB) を `fm7kanji.h` の中に入れてください。

# 接続

VGA コネクタとの接続は以下の通りです

- GPIO0: HSYNC
- GPIO1: VSYNC
- GPIO2: Blue
- GPIO3: Red
- GPIO4: Green
- GND: GND

RGB の出力と VGA コネクタの間には 330 Ωの抵抗を直列に入れてください。

音声出力は GPIO5 を使っていますが、そのままではかなり音が小さいので何らかのアンプを通した方がよいと思います。

# 画面出力

PIO を使った VGA 画面出力を行っています。
元のライブラリが、640x480 出力だったものを、メモリ節約のため 640x400 出力に変更しています。

カラーパレットの操作やスクロール時に、実機では不要な全画面の書き換えが必要になる場合があります。
そのようなソフトでは動作が遅くなります。

# キーボード

USB ホスト機能を使っています。Pico の USB ポートに OTG アダプターをつけて USB キーボードを接続します。
FM-7 に存在しないキーは以下のようになっています。

- GRAPH -> ALT
- かな -> カタカナ/ひらがな
- Break -> Pause/Break
- NumLock -> テンキーの`=`
- End -> DUP
- PageUp -> EL
- PageDown -> CLS

F12 キーを押すとメニュー画面になります。
ここで、セーブ＆ロード用のファイル選択、およびリセットを行うことができます。

# カセット

カセットのセーブ＆ロードは、UART 経由の出力と LittleFS を使ってフラッシュへ保存ができます。
通常 UART 経由で入出力されるようになっていますが、
メニューでファイルを選択すると LittleFS 経由になります。

UART経由の場合、単にデータを 16進数にしたものが使用されます。
流し込む場合は、適当なディレイ(1ms くらい？)を入れてください。
ボーレートの切り替えは無視されます。
常に 115200 bps で通信します。

なお、LittleFS の容量は 0x80000 からの 512KiB を確保していますが、これは 1M フラッシュの RP2040-A で使用することを想定しているためです。
2M フラッシュの純正 Pico を使う場合はもっと容量を増やすことができます。

なお littleFS のバックアップは SWD を接続して OpenOCD を使ってください。

現状、巨大なファイルの LittleFS 経由の読み書きは不安定な時があります。

# ライセンスなど

このプロフラムは Pico SDK 以外に、以下のライブラリを利用しています。

- [MC6809 エミュレータ(一部改変)](https://github.com/spc476/mc6809)
- [VGA ライブラリ(一部改変)](https://github.com/vha3/Hunter-Adams-RP2040-Demos/tree/master/VGA_Graphics)
- [LittleFS](https://github.com/littlefs-project/littlefs)

# Gallary
