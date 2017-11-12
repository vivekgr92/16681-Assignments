<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="8.3.2">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="50" name="dxf" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="53" name="tGND_GNDA" color="7" fill="9" visible="no" active="no"/>
<layer number="54" name="bGND_GNDA" color="1" fill="9" visible="no" active="no"/>
<layer number="56" name="wert" color="7" fill="1" visible="no" active="no"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="100" name="Muster" color="7" fill="1" visible="no" active="no"/>
<layer number="101" name="Patch_Top" color="12" fill="4" visible="yes" active="yes"/>
<layer number="102" name="Vscore" color="7" fill="1" visible="yes" active="yes"/>
<layer number="104" name="Name" color="7" fill="1" visible="yes" active="yes"/>
<layer number="116" name="Patch_BOT" color="9" fill="4" visible="yes" active="yes"/>
<layer number="121" name="_tsilk" color="7" fill="1" visible="yes" active="yes"/>
<layer number="122" name="_bsilk" color="7" fill="1" visible="yes" active="yes"/>
<layer number="151" name="HeatSink" color="7" fill="1" visible="yes" active="yes"/>
<layer number="200" name="200bmp" color="1" fill="10" visible="no" active="no"/>
<layer number="201" name="201bmp" color="2" fill="1" visible="no" active="no"/>
<layer number="202" name="202bmp" color="3" fill="1" visible="no" active="no"/>
<layer number="250" name="Descript" color="3" fill="1" visible="yes" active="yes"/>
<layer number="251" name="SMDround" color="12" fill="11" visible="no" active="no"/>
<layer number="254" name="OrgLBR" color="13" fill="1" visible="no" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="TinyGadget">
<packages>
<package name="DIL08">
<description>&lt;B&gt;Dual In Line&lt;/B&gt;</description>
<wire x1="-5.08" y1="-0.635" x2="-5.08" y2="0.635" width="0.1524" layer="21" curve="180"/>
<wire x1="-5.08" y1="-0.635" x2="-5.08" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-2.794" x2="5.08" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-2.794" x2="5.08" y2="2.794" width="0.1524" layer="21"/>
<wire x1="5.08" y1="2.794" x2="-5.08" y2="2.794" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="2.794" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<pad name="1" x="-3.81" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="2" x="-1.27" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="3" x="1.27" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="4" x="3.81" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="5" x="3.81" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="6" x="1.27" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="7" x="-1.27" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="8" x="-3.81" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<text x="-5.3594" y="-2.54" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="-3.81" y="-0.9906" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="8S1">
<description>&lt;b&gt;8S1&lt;/b&gt; 8-lead (0.150" Wide Body)&lt;p&gt;
Plastic Gull Wing Small Outline (JEDEC SOIC)&lt;br&gt;
Source: http://www.atmel.com/dyn/resources/prod_documents/2535S.pdf</description>
<wire x1="-2.425" y1="1.675" x2="-2.175" y2="1.925" width="0.1524" layer="51" curve="-90"/>
<wire x1="2.175" y1="1.925" x2="2.425" y2="1.675" width="0.1524" layer="51" curve="-90"/>
<wire x1="2.175" y1="-1.925" x2="2.425" y2="-1.675" width="0.1524" layer="51" curve="90"/>
<wire x1="-2.425" y1="-1.675" x2="-2.175" y2="-1.925" width="0.1524" layer="51" curve="90"/>
<wire x1="-1.59" y1="1.925" x2="-0.95" y2="1.925" width="0.1524" layer="51"/>
<wire x1="-2.165" y1="1.925" x2="2.185" y2="1.925" width="0.1524" layer="51"/>
<wire x1="0.95" y1="1.925" x2="1.59" y2="1.925" width="0.1524" layer="51"/>
<wire x1="2.21" y1="1.925" x2="2.185" y2="1.925" width="0.1524" layer="21"/>
<wire x1="2.2" y1="-1.925" x2="2.185" y2="-1.925" width="0.1524" layer="51"/>
<wire x1="2.185" y1="-1.925" x2="2.155" y2="-1.925" width="0.1524" layer="51"/>
<wire x1="1.59" y1="-1.925" x2="0.94" y2="-1.925" width="0.1524" layer="51"/>
<wire x1="0.32" y1="-1.925" x2="-0.33" y2="-1.925" width="0.1524" layer="51"/>
<wire x1="2.185" y1="-1.925" x2="-2.165" y2="-1.925" width="0.1524" layer="51"/>
<wire x1="-2.425" y1="1.675" x2="-2.425" y2="-1.665" width="0.1524" layer="21"/>
<wire x1="2.425" y1="-1.675" x2="2.425" y2="1.675" width="0.1524" layer="21"/>
<circle x="-1.42" y="-1.115" radius="0.5" width="0.0508" layer="21"/>
<smd name="1" x="-1.905" y="-2.654" dx="0.6" dy="1.6" layer="1"/>
<smd name="2" x="-0.645" y="-2.654" dx="0.6" dy="1.6" layer="1"/>
<smd name="3" x="0.625" y="-2.654" dx="0.6" dy="1.6" layer="1"/>
<smd name="4" x="1.895" y="-2.654" dx="0.6" dy="1.6" layer="1"/>
<smd name="8" x="-1.905" y="2.654" dx="0.6" dy="1.6" layer="1"/>
<smd name="7" x="-0.635" y="2.654" dx="0.6" dy="1.6" layer="1"/>
<smd name="6" x="0.635" y="2.654" dx="0.6" dy="1.6" layer="1"/>
<smd name="5" x="1.905" y="2.654" dx="0.6" dy="1.6" layer="1"/>
<text x="-2.8575" y="-2.159" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="4.064" y="-2.159" size="1.27" layer="27" ratio="10" rot="R90">&gt;VALUE</text>
<rectangle x1="-2.08" y1="1.975" x2="-1.73" y2="3.075" layer="51"/>
<rectangle x1="-0.81" y1="1.975" x2="-0.46" y2="3.075" layer="51"/>
<rectangle x1="0.46" y1="1.975" x2="0.81" y2="3.075" layer="51"/>
<rectangle x1="1.73" y1="1.975" x2="2.08" y2="3.075" layer="51"/>
<rectangle x1="1.72" y1="-3.075" x2="2.07" y2="-1.975" layer="51"/>
<rectangle x1="0.45" y1="-3.075" x2="0.8" y2="-1.975" layer="51"/>
<rectangle x1="-0.82" y1="-3.075" x2="-0.47" y2="-1.975" layer="51"/>
<rectangle x1="-2.08" y1="-3.075" x2="-1.73" y2="-1.975" layer="51"/>
</package>
<package name="0204/7">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 7.5 mm</description>
<wire x1="3.81" y1="0" x2="2.921" y2="0" width="0.508" layer="51"/>
<wire x1="-3.81" y1="0" x2="-2.921" y2="0" width="0.508" layer="51"/>
<wire x1="-2.54" y1="0.762" x2="-2.286" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="-0.762" x2="-2.286" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="-1.016" x2="2.54" y2="-0.762" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="1.016" x2="2.54" y2="0.762" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="-0.762" x2="-2.54" y2="0.762" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="1.016" x2="-1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-1.778" y1="0.889" x2="-1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="-1.016" x2="-1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-1.778" y1="-0.889" x2="-1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.889" x2="1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.889" x2="-1.778" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.889" x2="1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.889" x2="-1.778" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="2.286" y1="1.016" x2="1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.286" y1="-1.016" x2="1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.762" x2="2.54" y2="0.762" width="0.1524" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.54" y="1.2954" size="0.9906" layer="25" ratio="12">&gt;NAME</text>
<text x="-1.6256" y="-0.4826" size="0.9906" layer="27" ratio="12">&gt;VALUE</text>
<rectangle x1="2.54" y1="-0.254" x2="2.921" y2="0.254" layer="21"/>
<rectangle x1="-2.921" y1="-0.254" x2="-2.54" y2="0.254" layer="21"/>
</package>
<package name="R0402">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.245" y1="0.224" x2="0.245" y2="0.224" width="0.1524" layer="51"/>
<wire x1="0.245" y1="-0.224" x2="-0.245" y2="-0.224" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.483" x2="1.473" y2="0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.483" x2="1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.483" x2="-1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.483" x2="-1.473" y2="0.483" width="0.0508" layer="39"/>
<smd name="1" x="-0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<smd name="2" x="0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.554" y1="-0.3048" x2="-0.254" y2="0.2951" layer="51"/>
<rectangle x1="0.2588" y1="-0.3048" x2="0.5588" y2="0.2951" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="LED5MM">
<description>&lt;B&gt;LED&lt;/B&gt;&lt;p&gt;
5 mm, round</description>
<wire x1="2.54" y1="-1.905" x2="2.54" y2="1.905" width="0.2032" layer="21"/>
<wire x1="2.54" y1="-1.905" x2="2.54" y2="1.905" width="0.254" layer="21" curve="-286.260205" cap="flat"/>
<wire x1="-1.143" y1="0" x2="0" y2="1.143" width="0.1524" layer="51" curve="-90" cap="flat"/>
<wire x1="0" y1="-1.143" x2="1.143" y2="0" width="0.1524" layer="51" curve="90" cap="flat"/>
<wire x1="-1.651" y1="0" x2="0" y2="1.651" width="0.1524" layer="51" curve="-90" cap="flat"/>
<wire x1="0" y1="-1.651" x2="1.651" y2="0" width="0.1524" layer="51" curve="90" cap="flat"/>
<wire x1="-2.159" y1="0" x2="0" y2="2.159" width="0.1524" layer="51" curve="-90" cap="flat"/>
<wire x1="0" y1="-2.159" x2="2.159" y2="0" width="0.1524" layer="51" curve="90" cap="flat"/>
<circle x="0" y="0" radius="2.54" width="0.1524" layer="21"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="3.175" y="0.5334" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="3.2004" y="-1.8034" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="CHIPLED_0603">
<description>&lt;b&gt;CHIPLED&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LG_LY Q971.pdf</description>
<wire x1="-0.3" y1="0.8" x2="0.3" y2="0.8" width="0.1016" layer="51" curve="170.055574"/>
<wire x1="-0.275" y1="-0.825" x2="0.275" y2="-0.825" width="0.0508" layer="51" curve="-180"/>
<wire x1="-0.4" y1="0.375" x2="-0.4" y2="-0.35" width="0.1016" layer="51"/>
<wire x1="0.4" y1="0.35" x2="0.4" y2="-0.35" width="0.1016" layer="51"/>
<circle x="-0.35" y="0.625" radius="0.075" width="0.0508" layer="51"/>
<smd name="C" x="0" y="0.75" dx="0.8" dy="0.8" layer="1"/>
<smd name="A" x="0" y="-0.75" dx="0.8" dy="0.8" layer="1"/>
<text x="-0.635" y="-1.27" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="1.905" y="-1.27" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.45" y1="0.7" x2="-0.25" y2="0.85" layer="51"/>
<rectangle x1="-0.275" y1="0.55" x2="-0.225" y2="0.6" layer="51"/>
<rectangle x1="-0.45" y1="0.35" x2="-0.4" y2="0.725" layer="51"/>
<rectangle x1="0.25" y1="0.55" x2="0.45" y2="0.85" layer="51"/>
<rectangle x1="-0.45" y1="0.35" x2="0.45" y2="0.575" layer="51"/>
<rectangle x1="-0.45" y1="-0.85" x2="-0.25" y2="-0.35" layer="51"/>
<rectangle x1="0.25" y1="-0.85" x2="0.45" y2="-0.35" layer="51"/>
<rectangle x1="-0.275" y1="-0.575" x2="0.275" y2="-0.35" layer="51"/>
<rectangle x1="-0.275" y1="-0.65" x2="-0.175" y2="-0.55" layer="51"/>
<rectangle x1="0.175" y1="-0.65" x2="0.275" y2="-0.55" layer="51"/>
<rectangle x1="-0.125" y1="0" x2="0.125" y2="0.25" layer="21"/>
</package>
<package name="BA2032">
<wire x1="0" y1="8" x2="28" y2="8" width="0.127" layer="21"/>
<wire x1="28" y1="8" x2="28" y2="-8" width="0.127" layer="21"/>
<wire x1="28" y1="-8" x2="0" y2="-8" width="0.127" layer="21"/>
<wire x1="0" y1="-8" x2="0" y2="8" width="0.127" layer="21"/>
<wire x1="26" y1="5" x2="26" y2="3" width="0.127" layer="21"/>
<wire x1="25" y1="4" x2="27" y2="4" width="0.127" layer="21"/>
<wire x1="1" y1="2" x2="3" y2="2" width="0.127" layer="21"/>
<pad name="P$1" x="0.55" y="0" drill="0.9906" shape="octagon"/>
<pad name="P$2" x="27.45" y="1.27" drill="0.9906" shape="octagon"/>
<pad name="P$3" x="27.45" y="-1.27" drill="0.9906" shape="octagon"/>
</package>
<package name="BA2032SM">
<wire x1="-1.27" y1="8.001" x2="-1.27" y2="-8.001" width="0.127" layer="21"/>
<wire x1="-1.27" y1="-8.001" x2="30.734" y2="-8.001" width="0.127" layer="21"/>
<wire x1="30.734" y1="-8.001" x2="30.734" y2="8.001" width="0.127" layer="21"/>
<wire x1="30.734" y1="8.001" x2="-1.27" y2="8.001" width="0.127" layer="21"/>
<smd name="3" x="0" y="0" dx="2.5908" dy="3.556" layer="1"/>
<smd name="1" x="29.3116" y="0" dx="2.5908" dy="3.556" layer="1"/>
<smd name="2" x="0" y="-5.08" dx="2.5908" dy="3.556" layer="1"/>
<text x="0" y="10.16" size="1.27" layer="25">&gt;NAME</text>
<text x="0" y="8.89" size="1.27" layer="27">&gt;VALUE</text>
<text x="0" y="2.54" size="1.27" layer="27">+</text>
<text x="29.21" y="2.54" size="1.27" layer="27">-</text>
</package>
<package name="C025-025X050">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 2.5 x 5 mm</description>
<wire x1="-2.159" y1="1.27" x2="2.159" y2="1.27" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.27" x2="-2.159" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.27" x2="2.413" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.016" x2="-2.159" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.27" x2="2.413" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.016" x2="-2.159" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C0402">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-0.245" y1="0.224" x2="0.245" y2="0.224" width="0.1524" layer="51"/>
<wire x1="0.245" y1="-0.224" x2="-0.245" y2="-0.224" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.483" x2="1.473" y2="0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.483" x2="1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.483" x2="-1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.483" x2="-1.473" y2="0.483" width="0.0508" layer="39"/>
<smd name="1" x="-0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<smd name="2" x="0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.554" y1="-0.3048" x2="-0.254" y2="0.2951" layer="51"/>
<rectangle x1="0.2588" y1="-0.3048" x2="0.5588" y2="0.2951" layer="51"/>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
</package>
<package name="6P2R0.1IN">
<wire x1="-1.27" y1="3.81" x2="-1.27" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-1.27" y1="-1.27" x2="6.35" y2="-1.27" width="0.127" layer="21"/>
<wire x1="6.35" y1="-1.27" x2="6.35" y2="3.81" width="0.127" layer="21"/>
<wire x1="6.35" y1="3.81" x2="-1.27" y2="3.81" width="0.127" layer="21"/>
<pad name="1" x="0" y="0" drill="1.016" diameter="1.6764" shape="square"/>
<pad name="2" x="0" y="2.54" drill="1.016" diameter="1.6764" shape="octagon"/>
<pad name="3" x="2.54" y="0" drill="1.016" diameter="1.6764" shape="octagon"/>
<pad name="4" x="2.54" y="2.54" drill="1.016" diameter="1.6764" shape="octagon"/>
<pad name="5" x="5.08" y="0" drill="1.016" diameter="1.6764" shape="octagon"/>
<pad name="6" x="5.08" y="2.54" drill="1.016" diameter="1.6764" shape="octagon"/>
<text x="-1.27" y="3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="25">&gt;VALUE</text>
</package>
<package name="SM6PIN">
<wire x1="-0.508" y1="2.794" x2="-0.762" y2="2.794" width="0.127" layer="25"/>
<wire x1="-0.762" y1="2.794" x2="-0.762" y2="-1.524" width="0.127" layer="25"/>
<wire x1="-0.762" y1="-1.524" x2="-0.508" y2="-1.524" width="0.127" layer="25"/>
<wire x1="3.048" y1="2.794" x2="3.302" y2="2.794" width="0.127" layer="25"/>
<wire x1="3.302" y1="2.794" x2="3.302" y2="-1.524" width="0.127" layer="25"/>
<wire x1="3.302" y1="-1.524" x2="3.048" y2="-1.524" width="0.127" layer="25"/>
<smd name="1" x="0" y="-1.27" dx="0.7366" dy="1.8542" layer="1"/>
<smd name="3" x="1.27" y="-1.27" dx="0.7366" dy="1.8542" layer="1"/>
<smd name="5" x="2.54" y="-1.27" dx="0.7366" dy="1.8542" layer="1"/>
<smd name="2" x="0" y="2.794" dx="0.7366" dy="1.8542" layer="1"/>
<smd name="4" x="1.27" y="2.794" dx="0.7366" dy="1.8542" layer="1"/>
<smd name="6" x="2.54" y="2.794" dx="0.7366" dy="1.8542" layer="1"/>
<text x="-1.524" y="-1.524" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="5.08" y="-1.524" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<hole x="0.635" y="0.762" drill="0.889"/>
<hole x="1.905" y="0.762" drill="0.889"/>
</package>
<package name="PUSHCON3">
<wire x1="8.7" y1="7.3" x2="-3.3" y2="7.3" width="0.127" layer="21"/>
<wire x1="-3.3" y1="7.3" x2="-3.3" y2="-4.7" width="0.127" layer="21"/>
<wire x1="-3.3" y1="-4.7" x2="8.7" y2="-4.7" width="0.127" layer="21"/>
<wire x1="8.7" y1="-4.7" x2="8.7" y2="7.3" width="0.127" layer="21"/>
<pad name="1" x="0" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="3.5" y="5" drill="1.1" shape="octagon"/>
<pad name="3" x="7" y="0" drill="1.1" shape="octagon"/>
<text x="-3.7" y="-3.8" size="1.27" layer="25" rot="R90">&gt;NAME</text>
</package>
<package name="SM3PIN">
<wire x1="-0.508" y1="-0.508" x2="-0.762" y2="-0.508" width="0.127" layer="21"/>
<wire x1="-0.762" y1="-0.508" x2="-0.762" y2="-2.286" width="0.127" layer="21"/>
<wire x1="-0.762" y1="-2.286" x2="0.762" y2="-2.286" width="0.127" layer="21"/>
<wire x1="1.778" y1="-2.286" x2="3.302" y2="-2.286" width="0.127" layer="21"/>
<wire x1="3.302" y1="-2.286" x2="3.302" y2="-0.508" width="0.127" layer="21"/>
<wire x1="3.302" y1="-0.508" x2="3.048" y2="-0.508" width="0.127" layer="21"/>
<wire x1="2.032" y1="-0.508" x2="0.508" y2="-0.508" width="0.127" layer="21"/>
<smd name="1" x="0" y="0" dx="0.7366" dy="1.8542" layer="1"/>
<smd name="3" x="2.54" y="0" dx="0.7366" dy="1.8542" layer="1"/>
<smd name="2" x="1.27" y="-2.794" dx="0.7366" dy="1.8542" layer="1"/>
<text x="-1.27" y="-2.794" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="4.826" y="-2.794" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="ATTINY13A">
<wire x1="-7.62" y1="12.7" x2="-7.62" y2="-10.16" width="0.254" layer="94"/>
<wire x1="-7.62" y1="-10.16" x2="12.7" y2="-10.16" width="0.254" layer="94"/>
<wire x1="12.7" y1="-10.16" x2="12.7" y2="12.7" width="0.254" layer="94"/>
<wire x1="12.7" y1="12.7" x2="-7.62" y2="12.7" width="0.254" layer="94"/>
<text x="-7.62" y="17.78" size="1.27" layer="95">&gt;NAME</text>
<text x="-7.62" y="15.24" size="1.27" layer="96">&gt;VALUE</text>
<pin name="VCC" x="-12.7" y="10.16" length="middle"/>
<pin name="GND" x="-12.7" y="-7.62" length="middle"/>
<pin name="B0/MOSI/PWMA" x="17.78" y="7.62" length="middle" rot="R180"/>
<pin name="B1/MISO/PWMB" x="17.78" y="5.08" length="middle" rot="R180"/>
<pin name="B2/SCK/ADC1" x="17.78" y="2.54" length="middle" rot="R180"/>
<pin name="B3/ADC3" x="17.78" y="0" length="middle" rot="R180"/>
<pin name="B4/ADC2" x="17.78" y="-2.54" length="middle" rot="R180"/>
<pin name="B5/!RESET" x="17.78" y="-5.08" length="middle" rot="R180"/>
</symbol>
<symbol name="RESISTOR">
<wire x1="-2.54" y1="0" x2="-2.159" y2="1.016" width="0.2032" layer="94"/>
<wire x1="-2.159" y1="1.016" x2="-1.524" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="-1.524" y1="-1.016" x2="-0.889" y2="1.016" width="0.2032" layer="94"/>
<wire x1="-0.889" y1="1.016" x2="-0.254" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="-0.254" y1="-1.016" x2="0.381" y2="1.016" width="0.2032" layer="94"/>
<wire x1="0.381" y1="1.016" x2="1.016" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="1.016" y1="-1.016" x2="1.651" y2="1.016" width="0.2032" layer="94"/>
<wire x1="1.651" y1="1.016" x2="2.286" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="2.286" y1="-1.016" x2="2.54" y2="0" width="0.2032" layer="94"/>
<text x="-3.81" y="1.4986" size="1.778" layer="95">&gt;NAME</text>
<text x="-3.81" y="-3.302" size="1.778" layer="96">&gt;VALUE</text>
<pin name="2" x="5.08" y="0" visible="off" length="short" swaplevel="1" rot="R180"/>
<pin name="1" x="-5.08" y="0" visible="off" length="short" swaplevel="1"/>
</symbol>
<symbol name="GND">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
<symbol name="LED">
<wire x1="1.27" y1="0" x2="0" y2="-2.54" width="0.254" layer="94"/>
<wire x1="0" y1="-2.54" x2="-1.27" y2="0" width="0.254" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="0" y2="-2.54" width="0.254" layer="94"/>
<wire x1="0" y1="-2.54" x2="-1.27" y2="-2.54" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="0" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="-2.032" y1="-0.762" x2="-3.429" y2="-2.159" width="0.1524" layer="94"/>
<wire x1="-1.905" y1="-1.905" x2="-3.302" y2="-3.302" width="0.1524" layer="94"/>
<text x="3.556" y="-4.572" size="1.778" layer="95" rot="R90">&gt;NAME</text>
<text x="5.715" y="-4.572" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="C" x="0" y="-5.08" visible="off" length="short" rot="R90"/>
<pin name="A" x="0" y="2.54" visible="off" length="short" rot="R270"/>
<polygon width="0.1524" layer="94">
<vertex x="-3.429" y="-2.159"/>
<vertex x="-3.048" y="-1.27"/>
<vertex x="-2.54" y="-1.778"/>
</polygon>
<polygon width="0.1524" layer="94">
<vertex x="-3.302" y="-3.302"/>
<vertex x="-2.921" y="-2.413"/>
<vertex x="-2.413" y="-2.921"/>
</polygon>
</symbol>
<symbol name="VCC">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="VCC" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="BA2032">
<wire x1="2.54" y1="6.35" x2="2.54" y2="3.81" width="0.254" layer="94"/>
<wire x1="1.27" y1="5.08" x2="3.81" y2="5.08" width="0.254" layer="94"/>
<wire x1="2.54" y1="-1.27" x2="2.54" y2="-3.81" width="0.254" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="3.81" y2="-2.54" width="0.254" layer="94"/>
<wire x1="-6.35" y1="-2.54" x2="-3.81" y2="-2.54" width="0.254" layer="94"/>
<wire x1="-7.62" y1="-6.35" x2="-7.62" y2="7.62" width="0.254" layer="95"/>
<wire x1="-7.62" y1="7.62" x2="5.08" y2="7.62" width="0.254" layer="95"/>
<wire x1="5.08" y1="7.62" x2="5.08" y2="-6.35" width="0.254" layer="95"/>
<wire x1="5.08" y1="-6.35" x2="-7.62" y2="-6.35" width="0.254" layer="95"/>
<text x="-6.35" y="8.89" size="1.27" layer="95">&gt;NAME</text>
<pin name="P$1" x="-12.7" y="-2.54" visible="off" length="middle"/>
<pin name="P$2" x="10.16" y="5.08" visible="off" length="middle" rot="R180"/>
<pin name="P$3" x="10.16" y="-2.54" visible="off" length="middle" rot="R180"/>
</symbol>
<symbol name="CAPACITOR">
<wire x1="-2.54" y1="0" x2="2.54" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="-1.016" x2="0" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="0" y1="-1" x2="2.4892" y2="-1.8542" width="0.254" layer="94" curve="-37.878202" cap="flat"/>
<wire x1="-2.4668" y1="-1.8504" x2="0" y2="-1.0161" width="0.254" layer="94" curve="-37.373024" cap="flat"/>
<text x="1.016" y="0.635" size="1.778" layer="95">&gt;NAME</text>
<text x="1.016" y="-4.191" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="0" y="2.54" visible="off" length="short" swaplevel="1" rot="R270"/>
<pin name="2" x="0" y="-5.08" visible="off" length="short" swaplevel="1" rot="R90"/>
</symbol>
<symbol name="CON6P2R">
<wire x1="3.81" y1="-5.08" x2="-3.81" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="-3.81" y1="5.08" x2="-3.81" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="-3.81" y1="5.08" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-5.08" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<wire x1="-2.54" y1="-2.54" x2="-1.27" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="0" x2="-1.27" y2="0" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="2.54" x2="-1.27" y2="2.54" width="0.6096" layer="94"/>
<wire x1="2.54" y1="-2.54" x2="1.27" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="2.54" y1="0" x2="1.27" y2="0" width="0.6096" layer="94"/>
<wire x1="2.54" y1="2.54" x2="1.27" y2="2.54" width="0.6096" layer="94"/>
<text x="-3.81" y="-7.62" size="1.778" layer="96">&gt;VALUE</text>
<text x="-3.81" y="5.842" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="-7.62" y="2.54" visible="pad" length="middle"/>
<pin name="3" x="-7.62" y="0" visible="pad" length="middle"/>
<pin name="5" x="-7.62" y="-2.54" visible="pad" length="middle"/>
<pin name="2" x="7.62" y="2.54" visible="pad" length="middle" rot="R180"/>
<pin name="4" x="7.62" y="0" visible="pad" length="middle" rot="R180"/>
<pin name="6" x="7.62" y="-2.54" visible="pad" length="middle" rot="R180"/>
</symbol>
<symbol name="CON3">
<wire x1="-2.54" y1="5.08" x2="-2.54" y2="-5.08" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-5.08" x2="2.54" y2="-5.08" width="0.254" layer="94"/>
<wire x1="2.54" y1="-5.08" x2="2.54" y2="5.08" width="0.254" layer="94"/>
<wire x1="2.54" y1="5.08" x2="-2.54" y2="5.08" width="0.254" layer="94"/>
<text x="-10.16" y="6.35" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="-7.62" y="2.54" visible="pin" length="middle"/>
<pin name="2" x="-7.62" y="0" visible="pin" length="middle"/>
<pin name="3" x="-7.62" y="-2.54" visible="pin" length="middle"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="ATTINY25">
<gates>
<gate name="G$1" symbol="ATTINY13A" x="-2.54" y="0"/>
</gates>
<devices>
<device name="TH_ATTINY" package="DIL08">
<connects>
<connect gate="G$1" pin="B0/MOSI/PWMA" pad="5"/>
<connect gate="G$1" pin="B1/MISO/PWMB" pad="6"/>
<connect gate="G$1" pin="B2/SCK/ADC1" pad="7"/>
<connect gate="G$1" pin="B3/ADC3" pad="2"/>
<connect gate="G$1" pin="B4/ADC2" pad="3"/>
<connect gate="G$1" pin="B5/!RESET" pad="1"/>
<connect gate="G$1" pin="GND" pad="4"/>
<connect gate="G$1" pin="VCC" pad="8"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SM_ATTINY" package="8S1">
<connects>
<connect gate="G$1" pin="B0/MOSI/PWMA" pad="5"/>
<connect gate="G$1" pin="B1/MISO/PWMB" pad="6"/>
<connect gate="G$1" pin="B2/SCK/ADC1" pad="7"/>
<connect gate="G$1" pin="B3/ADC3" pad="2"/>
<connect gate="G$1" pin="B4/ADC2" pad="3"/>
<connect gate="G$1" pin="B5/!RESET" pad="1"/>
<connect gate="G$1" pin="GND" pad="4"/>
<connect gate="G$1" pin="VCC" pad="8"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="RESISTOR">
<gates>
<gate name="G$1" symbol="RESISTOR" x="2.54" y="0"/>
</gates>
<devices>
<device name="TH_RESISTOR" package="0204/7">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SMRESISTOR" package="R0402">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="GND" prefix="GND">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="LED">
<gates>
<gate name="G$1" symbol="LED" x="0" y="2.54"/>
</gates>
<devices>
<device name="TH_LED" package="LED5MM">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SMLED" package="CHIPLED_0603">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="VCC" prefix="P+">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="VCC" symbol="VCC" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="BATTERY">
<gates>
<gate name="G$1" symbol="BA2032" x="0" y="-2.54"/>
</gates>
<devices>
<device name="TH_BA2032" package="BA2032">
<connects>
<connect gate="G$1" pin="P$1" pad="P$1"/>
<connect gate="G$1" pin="P$2" pad="P$2"/>
<connect gate="G$1" pin="P$3" pad="P$3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SM_BA2032" package="BA2032SM">
<connects>
<connect gate="G$1" pin="P$1" pad="1"/>
<connect gate="G$1" pin="P$2" pad="2"/>
<connect gate="G$1" pin="P$3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="CAPACITOR">
<gates>
<gate name="G$1" symbol="CAPACITOR" x="0" y="0"/>
</gates>
<devices>
<device name="TH_CAP" package="C025-025X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SM_CAP" package="C0402">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="6PINCON_ISP" prefix="J" uservalue="yes">
<gates>
<gate name="G$1" symbol="CON6P2R" x="0" y="0"/>
</gates>
<devices>
<device name="TH_6PIN" package="6P2R0.1IN">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SM_6PIN" package="SM6PIN">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="3PINCON">
<gates>
<gate name="G$1" symbol="CON3" x="0" y="0"/>
</gates>
<devices>
<device name="TH_3PIN" package="PUSHCON3">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SM_3PIN" package="SM3PIN">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="frames" urn="urn:adsk.eagle:library:229">
<description>&lt;b&gt;Frames for Sheet and Layout&lt;/b&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="A4L-LOC" urn="urn:adsk.eagle:symbol:13874/1" library_version="1">
<wire x1="256.54" y1="3.81" x2="256.54" y2="8.89" width="0.1016" layer="94"/>
<wire x1="256.54" y1="8.89" x2="256.54" y2="13.97" width="0.1016" layer="94"/>
<wire x1="256.54" y1="13.97" x2="256.54" y2="19.05" width="0.1016" layer="94"/>
<wire x1="256.54" y1="19.05" x2="256.54" y2="24.13" width="0.1016" layer="94"/>
<wire x1="161.29" y1="3.81" x2="161.29" y2="24.13" width="0.1016" layer="94"/>
<wire x1="161.29" y1="24.13" x2="215.265" y2="24.13" width="0.1016" layer="94"/>
<wire x1="215.265" y1="24.13" x2="256.54" y2="24.13" width="0.1016" layer="94"/>
<wire x1="246.38" y1="3.81" x2="246.38" y2="8.89" width="0.1016" layer="94"/>
<wire x1="246.38" y1="8.89" x2="256.54" y2="8.89" width="0.1016" layer="94"/>
<wire x1="246.38" y1="8.89" x2="215.265" y2="8.89" width="0.1016" layer="94"/>
<wire x1="215.265" y1="8.89" x2="215.265" y2="3.81" width="0.1016" layer="94"/>
<wire x1="215.265" y1="8.89" x2="215.265" y2="13.97" width="0.1016" layer="94"/>
<wire x1="215.265" y1="13.97" x2="256.54" y2="13.97" width="0.1016" layer="94"/>
<wire x1="215.265" y1="13.97" x2="215.265" y2="19.05" width="0.1016" layer="94"/>
<wire x1="215.265" y1="19.05" x2="256.54" y2="19.05" width="0.1016" layer="94"/>
<wire x1="215.265" y1="19.05" x2="215.265" y2="24.13" width="0.1016" layer="94"/>
<text x="217.17" y="15.24" size="2.54" layer="94">&gt;DRAWING_NAME</text>
<text x="217.17" y="10.16" size="2.286" layer="94">&gt;LAST_DATE_TIME</text>
<text x="230.505" y="5.08" size="2.54" layer="94">&gt;SHEET</text>
<text x="216.916" y="4.953" size="2.54" layer="94">Sheet:</text>
<frame x1="0" y1="0" x2="260.35" y2="179.07" columns="6" rows="4" layer="94"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="A4L-LOC" urn="urn:adsk.eagle:component:13926/1" prefix="FRAME" uservalue="yes" library_version="1">
<description>&lt;b&gt;FRAME&lt;/b&gt;&lt;p&gt;
DIN A4, landscape with location and doc. field</description>
<gates>
<gate name="G$1" symbol="A4L-LOC" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="IC1" library="TinyGadget" deviceset="ATTINY25" device="TH_ATTINY" value="attiny25/45/85"/>
<part name="R2" library="TinyGadget" deviceset="RESISTOR" device="TH_RESISTOR" value="100"/>
<part name="R1" library="TinyGadget" deviceset="RESISTOR" device="TH_RESISTOR" value="4.7K"/>
<part name="GND1" library="TinyGadget" deviceset="GND" device=""/>
<part name="LED" library="TinyGadget" deviceset="LED" device="TH_LED" value=""/>
<part name="GND2" library="TinyGadget" deviceset="GND" device=""/>
<part name="P+2" library="TinyGadget" deviceset="VCC" device=""/>
<part name="BATTERY" library="TinyGadget" deviceset="BATTERY" device="TH_BA2032"/>
<part name="C1" library="TinyGadget" deviceset="CAPACITOR" device="TH_CAP" value="0.1uf"/>
<part name="GND3" library="TinyGadget" deviceset="GND" device=""/>
<part name="ISP" library="TinyGadget" deviceset="6PINCON_ISP" device="TH_6PIN"/>
<part name="P+3" library="TinyGadget" deviceset="VCC" device=""/>
<part name="GND4" library="TinyGadget" deviceset="GND" device=""/>
<part name="SENSE" library="TinyGadget" deviceset="3PINCON" device="TH_3PIN"/>
<part name="R3" library="TinyGadget" deviceset="RESISTOR" device="TH_RESISTOR" value="4.7K"/>
<part name="P+4" library="TinyGadget" deviceset="VCC" device=""/>
<part name="GND5" library="TinyGadget" deviceset="GND" device=""/>
<part name="FRAME1" library="frames" library_urn="urn:adsk.eagle:library:229" deviceset="A4L-LOC" device=""/>
<part name="P+1" library="TinyGadget" deviceset="VCC" device=""/>
</parts>
<sheets>
<sheet>
<plain>
<text x="218.44" y="17.78" size="1.778" layer="91">VIVEK GR</text>
</plain>
<instances>
<instance part="IC1" gate="G$1" x="129.54" y="93.98"/>
<instance part="R2" gate="G$1" x="157.48" y="101.6" smashed="yes">
<attribute name="NAME" x="156.21" y="103.0986" size="1.778" layer="95"/>
<attribute name="VALUE" x="156.21" y="98.298" size="1.778" layer="96"/>
</instance>
<instance part="R1" gate="G$1" x="78.74" y="111.76"/>
<instance part="GND1" gate="1" x="116.84" y="83.82"/>
<instance part="LED" gate="G$1" x="162.56" y="99.06" smashed="yes">
<attribute name="NAME" x="166.116" y="97.028" size="1.778" layer="95"/>
<attribute name="VALUE" x="168.275" y="94.488" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND2" gate="1" x="162.56" y="91.44"/>
<instance part="P+2" gate="VCC" x="73.66" y="81.28"/>
<instance part="BATTERY" gate="G$1" x="63.5" y="71.12"/>
<instance part="C1" gate="G$1" x="86.36" y="66.04"/>
<instance part="GND3" gate="1" x="50.8" y="30.48"/>
<instance part="ISP" gate="G$1" x="88.9" y="99.06"/>
<instance part="P+3" gate="VCC" x="104.14" y="119.38"/>
<instance part="GND4" gate="1" x="96.52" y="91.44"/>
<instance part="SENSE" gate="G$1" x="187.96" y="114.3"/>
<instance part="R3" gate="G$1" x="190.5" y="129.54"/>
<instance part="P+4" gate="VCC" x="205.74" y="134.62"/>
<instance part="GND5" gate="1" x="167.64" y="109.22"/>
<instance part="FRAME1" gate="G$1" x="0" y="-2.54"/>
<instance part="P+1" gate="VCC" x="111.76" y="114.3"/>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="0">
<segment>
<pinref part="LED" gate="G$1" pin="C"/>
<pinref part="GND2" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="GND"/>
<pinref part="GND1" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C1" gate="G$1" pin="2"/>
<wire x1="86.36" y1="60.96" x2="86.36" y2="40.64" width="0.1524" layer="91"/>
<wire x1="86.36" y1="40.64" x2="50.8" y2="40.64" width="0.1524" layer="91"/>
<pinref part="BATTERY" gate="G$1" pin="P$1"/>
<wire x1="50.8" y1="40.64" x2="50.8" y2="68.58" width="0.1524" layer="91"/>
<pinref part="GND3" gate="1" pin="GND"/>
<wire x1="50.8" y1="40.64" x2="50.8" y2="33.02" width="0.1524" layer="91"/>
<junction x="50.8" y="40.64"/>
</segment>
<segment>
<pinref part="ISP" gate="G$1" pin="6"/>
<pinref part="GND4" gate="1" pin="GND"/>
<wire x1="96.52" y1="96.52" x2="96.52" y2="93.98" width="0.1524" layer="91"/>
<label x="96.52" y="96.52" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="SENSE" gate="G$1" pin="2"/>
<wire x1="180.34" y1="114.3" x2="167.64" y2="114.3" width="0.1524" layer="91"/>
<pinref part="GND5" gate="1" pin="GND"/>
<wire x1="167.64" y1="114.3" x2="167.64" y2="111.76" width="0.1524" layer="91"/>
</segment>
</net>
<net name="VCC" class="0">
<segment>
<pinref part="C1" gate="G$1" pin="1"/>
<wire x1="86.36" y1="68.58" x2="73.66" y2="68.58" width="0.1524" layer="91"/>
<pinref part="BATTERY" gate="G$1" pin="P$3"/>
<pinref part="P+2" gate="VCC" pin="VCC"/>
<wire x1="73.66" y1="68.58" x2="73.66" y2="76.2" width="0.1524" layer="91"/>
<junction x="73.66" y="68.58"/>
<pinref part="BATTERY" gate="G$1" pin="P$2"/>
<wire x1="73.66" y1="76.2" x2="73.66" y2="78.74" width="0.1524" layer="91"/>
<junction x="73.66" y="76.2"/>
</segment>
<segment>
<pinref part="R1" gate="G$1" pin="2"/>
<wire x1="83.82" y1="111.76" x2="104.14" y2="111.76" width="0.1524" layer="91"/>
<wire x1="104.14" y1="111.76" x2="104.14" y2="101.6" width="0.1524" layer="91"/>
<pinref part="ISP" gate="G$1" pin="2"/>
<wire x1="96.52" y1="101.6" x2="104.14" y2="101.6" width="0.1524" layer="91"/>
<pinref part="P+3" gate="VCC" pin="VCC"/>
<wire x1="104.14" y1="116.84" x2="104.14" y2="111.76" width="0.1524" layer="91"/>
<junction x="104.14" y="111.76"/>
<label x="96.52" y="101.6" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="R3" gate="G$1" pin="2"/>
<pinref part="P+4" gate="VCC" pin="VCC"/>
<wire x1="195.58" y1="129.54" x2="205.74" y2="129.54" width="0.1524" layer="91"/>
<wire x1="205.74" y1="129.54" x2="205.74" y2="132.08" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="VCC"/>
<wire x1="116.84" y1="104.14" x2="111.76" y2="104.14" width="0.1524" layer="91"/>
<pinref part="P+1" gate="VCC" pin="VCC"/>
<wire x1="111.76" y1="104.14" x2="111.76" y2="111.76" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="R2" gate="G$1" pin="2"/>
<pinref part="LED" gate="G$1" pin="A"/>
</segment>
</net>
<net name="RESET" class="0">
<segment>
<pinref part="R1" gate="G$1" pin="1"/>
<wire x1="73.66" y1="111.76" x2="66.04" y2="111.76" width="0.1524" layer="91"/>
<wire x1="66.04" y1="111.76" x2="66.04" y2="96.52" width="0.1524" layer="91"/>
<pinref part="ISP" gate="G$1" pin="5"/>
<wire x1="66.04" y1="96.52" x2="81.28" y2="96.52" width="0.1524" layer="91"/>
<label x="73.66" y="96.52" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="B5/!RESET"/>
<wire x1="147.32" y1="88.9" x2="152.4" y2="88.9" width="0.1524" layer="91"/>
<label x="147.32" y="88.9" size="1.778" layer="95"/>
</segment>
</net>
<net name="SENSOR1" class="0">
<segment>
<pinref part="SENSE" gate="G$1" pin="1"/>
<wire x1="180.34" y1="116.84" x2="175.26" y2="116.84" width="0.1524" layer="91"/>
<wire x1="175.26" y1="116.84" x2="175.26" y2="129.54" width="0.1524" layer="91"/>
<pinref part="R3" gate="G$1" pin="1"/>
<wire x1="175.26" y1="129.54" x2="185.42" y2="129.54" width="0.1524" layer="91"/>
<label x="175.26" y="116.84" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="B3/ADC3"/>
<wire x1="147.32" y1="93.98" x2="152.4" y2="93.98" width="0.1524" layer="91"/>
<label x="147.32" y="93.98" size="1.778" layer="95"/>
</segment>
</net>
<net name="MISO" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="B1/MISO/PWMB"/>
<wire x1="147.32" y1="99.06" x2="152.4" y2="99.06" width="0.1524" layer="91"/>
<label x="147.32" y="99.06" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="ISP" gate="G$1" pin="1"/>
<wire x1="81.28" y1="101.6" x2="76.2" y2="101.6" width="0.1524" layer="91"/>
<label x="73.66" y="101.6" size="1.778" layer="95"/>
</segment>
</net>
<net name="SENSOR2" class="0">
<segment>
<pinref part="SENSE" gate="G$1" pin="3"/>
<wire x1="180.34" y1="111.76" x2="175.26" y2="111.76" width="0.1524" layer="91"/>
<label x="175.26" y="111.76" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="B4/ADC2"/>
<wire x1="147.32" y1="91.44" x2="152.4" y2="91.44" width="0.1524" layer="91"/>
<label x="147.32" y="91.44" size="1.778" layer="95"/>
</segment>
</net>
<net name="SCK" class="0">
<segment>
<pinref part="ISP" gate="G$1" pin="3"/>
<wire x1="81.28" y1="99.06" x2="76.2" y2="99.06" width="0.1524" layer="91"/>
<label x="73.66" y="99.06" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="B2/SCK/ADC1"/>
<wire x1="147.32" y1="96.52" x2="152.4" y2="96.52" width="0.1524" layer="91"/>
<label x="147.32" y="96.52" size="1.778" layer="95"/>
</segment>
</net>
<net name="MOSILED" class="0">
<segment>
<pinref part="ISP" gate="G$1" pin="4"/>
<wire x1="96.52" y1="99.06" x2="101.6" y2="99.06" width="0.1524" layer="91"/>
<label x="96.52" y="99.06" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="B0/MOSI/PWMA"/>
<pinref part="R2" gate="G$1" pin="1"/>
<wire x1="147.32" y1="101.6" x2="152.4" y2="101.6" width="0.1524" layer="91"/>
<label x="144.78" y="104.14" size="1.778" layer="95"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
<errors>
<approved hash="113,1,88.9,100.62,ISP,,,,,"/>
<approved hash="113,1,130.071,86.891,FRAME1,,,,,"/>
</errors>
</schematic>
</drawing>
<compatibility>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
</compatibility>
</eagle>
