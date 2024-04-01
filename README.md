Lab#3 任务说明

本次实验使用配套的线阵CCD模块，检测白底黑线场景中的黑线位置。
测试线阵CCD模块的信号特性并确定实验测试条件（各个步骤需做测试与分析）
评估模块的成像视角，构造合适的测试场景（即模拟合适尺寸的白色底板和合适宽度的黑线），使得线阵CCD采集到的128点数据能较好地体现出背景和导引线的特征；
确定合适的测试条件（主要是光照条件），使得线阵CCD不需要太长曝光时间即可获得具备良好特征的信号（曝光时间短信号数值整体偏小不利于可靠识别黑白线，而较长的曝光时间对应了较长的控制周期会直接影响控制性能）。
调试确定线阵CCD模块的工作状态（各个步骤需做测试与分析）
分析模块镜头“对焦”对信号结果的影响，确保线阵CCD模块处于较好的聚焦状态（各个模块都已初步做过对焦并用锁紧环做了固定，各组可选择重新对焦，或者能够从信号状态明确说明镜头已处于良好对焦状态的话也可以不用重新对焦只需结合信号结果具体说明即可）；
在确定了以上条件的基础上，通过测试确定初步的曝光时间（对应hsp_ccd_snapshot()函数中调用delay_1ms()所设定的延时参数调整）。
黑线位置识别与指示
基于所检测的线阵CCD信号数值特征，构思黑线位置识别的算法，并讨论场景中可能出现的一些特殊情况对算法的影响（包括黑点干扰、整体光线偏强或偏弱，等）；
对所构思的算法进行代码实现，并对正常场景条件以及所考虑的特殊情况进行实际测试，分析说明算法的有效性和可靠性；
动态显示黑线识别结果：
LCD上以标线或箭头方式（直观即可，具体形式不做限定）指示黑线位置；
同时通过核心板上的16位光柱显示黑线位置，可以选择“反显”（即黑线位置亮灯，白线区域灭灯），也可以选择“正显”（即黑线位置灭灯，白线区域亮灯），将线阵128点CCD的信号状态投射到16位光柱上。
附加功能要求
当黑线偏离中心位置较远时（比如黑线出现在线阵CCD信号的第20点之前或第108点之后），蜂鸣器间隔200ms短鸣；黑线位于线阵CCD信号的第21点至第107点之间时，蜂鸣器不鸣叫；
当实验场景中未识别到黑线时，蜂鸣器间隔100ms短鸣；如果连续未识别到黑线时间超过1秒，则蜂鸣器长鸣（此操作模拟小车的“丢线保护”功能）；
通过SW1拨码开关，将以上蜂鸣器鸣叫控制改为对LED1指示灯的同步控制（开发调试时为避免对周围同学的干扰可以选择使用LED1指示灯，实验测试时为获得良好的交互感受可选择使用蜂鸣器）。
结合调试分析，探讨将线阵CCD用于小车控制需要注意的问题；简要分析“动态曝光”的作用和实现思路。
 

说明：

1. 不提供实验报告模板，请参照一般报告的形式整理内容：

    1) 问题分析（线阵CCD应用场景、信号特性等）；

    2) 开发开发（算法构思与实现，数据测试与分析，代码优化，等）；

    3) 功能实现、结果测试及分析（可能存在的问题及解决方法）；

    4) 结论。

2. 本次实验可使用逐飞上位机辅助调试，但并非必须使用的方法，充分使用实验板上的液晶显示器和光柱、数码管等人机交互手段，也能很好地进行调试开发。

3. 实验报告为小组作业，每个小组共同提交一份。

4. 实验报告文件格式仅限word或pdf，本次实验不需录制视频，但需要在报告中附上清晰的图片以体现各个主要测试环节的细节；请将实验报告、单片机工程文件一起打包，在Canvas系统中提交。

