# ブロックと動作の対応

## ブロックと動作の定義

各ブロックの抵抗値と、動作の対応は、以下のように定義します。
- 2.2k : right turn
- 4.7k : left turn
-  10k : forward
-  22k : backward
-  47k : loop


## ブロックの配置位置と動作の実行順序の定義

- ブロックの配置位置と、動作の実行順序は以下のように定義します。
- SWを押すと、op1から順にop12へ、順に動作を実行します。実行するのは、置かれている最後のブロックまでです。
- 実行対象のブロックが"loop"ブロックの場合は、サブルーチンコールとなり、サブルーチンのloop1からloop4を順に実行します。loop1-4に置かれている最後のブロックまで実行したら、メインのopの実行へ戻り、loopの次のブロックの動作へ進みます。
- loop1-4に置かれたloopブロック(入れ子)は無視します。
- 各動作の間に、WAIT_AMONG_OPERATION(500ms)だけ待機します。
- 実行中にSWを押すと、実行を中断します。
- LEDの挙動は LED_definition.md を参照。

      col=0     1     2     3
row=0     loop1 loop2 loop3 loop4
row=1     op1   op2   op3   op4
row=2     op5   op6   op7   op8
row=3     op9   op10  op11  op12


## 各動作の定義
- forwardとbackwardは、それぞれ前進と後進の動作を行います。移動距離は STEP_MOVE [mm] で定義します(現在100mm)。
- left/right turnは、それぞれ左と右に、その場で90度回転する動作を行います。移動量(エンコーダのカウント値)は STEP_ROTATE で定義し、左右のタイヤが間隔(WHEEL_TREAD)を直径とする円周の1/4だけ逆向きに進む量として求めます。
- 移動距離からエンコーダのカウント値への換算には、以下の機体パラメータを使います。
  - WHEEL_DIAMETER : タイヤの直径 28mm
  - WHEEL_TREAD : 左右のタイヤの間隔 108mm
  - ENCODER_PULSES_PER_REV : タイヤ1回転あたりのエンコーダのパルス数 2086 (ギヤ比298 x 7pulse)


## エラー時の挙動

- ブロック実行は、実行順序の最後のブロックまで、とし、その途中にブロックが置かれていないところがある場合は、動作を一時停止し、その位置のLEDを LED_BLINK_CYCLE_ERROR で定義する時間（とりあえず200ms）で点滅させ、そこにブロックが置かれたら、LEDを点灯に変えて500ms待ってから、動作を継続
- この挙動は、loop1-4に対しても同様


## 定数の一覧 (include/program_runner.h)

| 定数 | 値 | 内容 |
|---|---|---|
| STEP_MOVE | 100.0 | forward/backwardの移動距離[mm] |
| STEP_ROTATE | (計算値) | 90度回転のカウント値 |
| LED_BLINK_CYCLE | 500 | 実行中ブロックのLED点滅周期[ms] |
| LED_BLINK_CYCLE_ERROR | 200 | ブロック未配置位置で待機中のLED点滅周期[ms] |
| WAIT_AMONG_OPERATION | 500 | 各動作の間の待機時間[ms] |
| WAIT_AFTER_BLOCK_PLACED | 500 | ブロック未配置位置にブロックが置かれてから動作を継続するまでの待機時間[ms] |
