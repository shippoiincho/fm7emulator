# FM-7 Emulator for Raspberry Pi Pico

# 概要

Raspberry Pi Pico で動作する、
FM-7 のエミュレータになります。
BML3 エミュレータの派生品です。

![screenshot](/pictures/screenshot00.jpg)


以下の機能を実装しています。

- メインメモリ 64KB
- PSG
- Beep 出力
- テープ出力
- FDD (一部)
- VRAM 48KB
- 作業用メモリ

- 漢字 ROM (オプション)

Pico を 225MHz で動かしていますが、ほぼ実機の速度が出ていると思います。

# ROM について

いつものように FM-7 実機の ROM が必要です。
`fm7rom_dummy.h` を `fm7rom.h` にリネームの上以下のデータを入れてください。

- F-BASIC
- Boot ROM (BASIC/DOS)
- サブシステム ROM

漢字ROM が必要な場合は、同様に漢字ROM の内容(128KB) を `fm7kanji.h` の中に入れてください。

# ビルド済みバイナリ

なお、ビルド済みバイナリを`prebuild` 以下に用意しています。


uf2 を、Pico に書き込むのと合わせて、ROM ファイルを Pico に置きます。

picotool を使う場合は、以下の通りで行けると思います。
(picotool は pico-sdk に含まれています)

```
$ picotool load -v -x fbasic300.rom -t bin -o 0x10070000
$ picotool load -v -x subsys_c.rom  -t bin -o 0x10078000
$ picotool load -v -x boot_bas.rom  -t bin -o 0x1007c000
$ picotool load -v -x boot_dos.rom  -t bin -o 0x1007e000
```

なお、漢字ROMが必要な方は、ソースからビルドしてください。

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
そのようなソフトでは動作が遅くなります。(Disk BASIC の DEMO など)

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

FM-7 のゲームは Break キーを使っているものが多いのですが、普通の USB キーボードの Break/Pause の場所は押しにくいので、
将来的にキーアサインは変更するかもしれません。

F12 キーを押すとメニュー画面になります。
ここで、セーブ＆ロード用のファイル選択、およびリセットを行うことができます。

![menu](/pictures/screenshot01.jpg)

# LittleFS について

テープやディスクイメージの保存に LittleFS を使っています。
既存のファイルの書き込みについては、
[こちらの記事を参照](https://shippoiincho.github.io/posts/39/)してください。

# テープ

テープのセーブ＆ロードは、UART 経由の出力と LittleFS を使ってフラッシュへ保存ができます。
通常 UART 経由で入出力されるようになっていますが、
メニューでファイルを選択すると LittleFS 経由になります。

UART経由の場合、単にデータを 16進数にしたものが使用されます。
流し込む場合は、適当なディレイ(1ms くらい？)を入れてください。
常に 115200 bps で通信します。

なお littleFS のバックアップは SWD を接続して OpenOCD を使ってください。
現状、巨大なファイルの LittleFS 経由の読み書きは不安定な時があります。

# フロッピー

2D のディスクを2台エミュレートしています。

FM-7のエミュレータでよく使われている D77ファイルを littlefs のイメージとして作成して、OpenOCD で Pico のフラッシュに書き込んで使います。
(ファイル名を 8.3 の範囲に抑えてください)
純正 Pico で 4 枚のディスクが収納できると思います。

セクターリードとセクターライトのみサポートしていますので、フォーマットとかトラックリードをするソフトは動作しません。
ディスクのアクセスに BootROM を使っているプログラムなら大抵は問題ないと思います。

# ライセンスなど

このプロフラムは Pico SDK 以外に、以下のライブラリを利用しています。

- [MC6809 エミュレータ(一部改変)](https://github.com/spc476/mc6809)
- [VGA ライブラリ(一部改変)](https://github.com/vha3/Hunter-Adams-RP2040-Demos/tree/master/VGA_Graphics)
- [LittleFS](https://github.com/littlefs-project/littlefs)

# 既知の問題

- FDC を直接制御するようなソフトはたぶん動きません。
- 6809 の未定義命令を使ったソフトは動作しません。
- FD の書き込みは非常に遅いうえに、充分な空き容量が必要です。
- スクロールレジスタの状態が反映されないことがあります。
- PicoSDK 2.0 や Pico2 での動作が不安定です
- メニュー画面を表示すると UART に空白文字が出力されます

# Gallary
![DEMO screenshot](/pictures/screenshot02.jpg)
![GAME screenshot](/pictures/screenshot03.jpg)
![OS9 screenshot](/pictures/screenshot04.jpg)