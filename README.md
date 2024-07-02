<H3>Schematic</H3>
<img src="https://github.com/dinhnam/StepmoterDrawingSimple/blob/master/Images/schematic.png" alt="drawing cnc schematic">
<H3>BOM</H3>
<p>KIT STM32f103c8t6<br>Module H bridge driver mini L298<br>Servo sg90<br>Two step motor bipolar 5v</p>
<H3>Image</H3>
<img src="https://github.com/dinhnam/StepmoterDrawingSimple/blob/master/Images/cnc_mini.jpg" alt="drawing cnc schematic">
<H3>Command UART</H3>
<p>MXY x y</p>
<p>Move pen to X-Axis and Y-Axis positions </p>
<p>LINE x1 y1 x2 y2</p>
<p>Move a line from position (x1,y1) to position (x2,y2)</p>
<p>CIRCLE x y R N</p>
<p>Draw a cirlce have center is (x,y) , radial is R, N is number of lines that make up a circle </p>
<p>PEN p_val</p>
<p>p_val = 0 => pen up. p_val = 1 => pen down.</p>
