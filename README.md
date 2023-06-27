# ub2qzqsm
シリアルポートに接続された、UBX-M8030-KT(u-blox) チップ搭載のGNSS受信機から  
QZSS(準天頂衛星: みちびき)から、災害・危機管理通報サービス「災危通報」(DC Report)を受信し  
それを$QZQSM (NMEA format)で出力します。

## 対応受信機
+ Windows 11 PC
+ 秋月のM-14541　GU-902MGG-USB (2600円のUSB-GPS) 

で動作確認しています。

https://akizukidenshi.com/catalog/g/gM-14541/


## 使い方
**このGNSS受信機は、初期状態ではL1Sの受信はオフになっており、また、災危通報が出力されるRXM-SFRBXの出力もオフになっていますが、本ツールが起動時に自動でオンにします。**

+ 空(天頂)のよく見えるところに、GNSS受信機を置いてください。窓の内側などでは受信できませんでした。(測位はできても、災危通報が取得できない状態になりやすいです。)
+ src/main.rsの```COM_PORT```を、お使いのPCで認識している番号に合わせます。
+ ```cargo run``` で起動します。
+ いくつかのACKの出力後、標準出力にQZQSMが出力されます。

うまく行かない場合は、u-centerなどで動作を確認してください。

例

```
PS D:\myprojects\rust\ub2qzqsm> cargo run
    Finished dev [unoptimized + debuginfo] target(s) in 0.02s
     Running `target\debug\ub2qzqsm.exe`
$QZQSM,56,53ADF36D0A0002CC2659860B30E966222D0045A710B4FB2862200013B401940*0D
$QZQSM,61,53ADF36D0A0002CC2659860B30E966222D0045A710B4FB2862200013B401940*09
$QZQSM,57,53ADF36D0A0002CC2659860B30E966222D0045A710B4FB2862200013B401940*0C
ACK
ACK
$QZQSM,57,C6ADC36D350006D350D1FD8DEC0800000000000000000000000000101A7D870*7E
$QZQSM,56,C6ADC36D350006D350D1FD8DEC0800000000000000000000000000101A7D870*7F
$QZQSM,61,C6ADC36D350006D350D1FD8DEC0800000000000000000000000000101A7D870*7B
$QZQSM,56,53ADF36D0A0002C3E8588ACB118162352C474588FCB1F4165DC000106AE7C68*78
$QZQSM,61,53ADF36D0A0002C3E8588ACB118162352C474588FCB1F4165DC000106AE7C68*7C
$QZQSM,57,53ADF36D0A0002C3E8588ACB118162352C474588FCB1F4165DC000106AE7C68*79
$QZQSM,56,9AADF36D0A0002CC2659860B30E966222D0045A710B4FB2862200010A59F238*08
$QZQSM,61,9AADF36D0A0002CC2659860B30E966222D0045A710B4FB2862200010A59F238*0C
$QZQSM,57,9AADF36D0A0002CC2659860B30E966222D0045A710B4FB2862200010A59F238*09
$QZQSM,56,C6ADF36D0A00051004A2710000000000000000000000000000000010C7474B4*00
$QZQSM,61,C6ADF36D0A00051004A2710000000000000000000000000000000010C7474B4*04
$QZQSM,57,C6ADF36D0A00051004A2710000000000000000000000000000000010C7474B4*01
$QZQSM,56,53ADC36D350006D350D1FD8DEC08000000000000000000000000001137581F4*7B
$QZQSM,61,53ADC36D350006D350D1FD8DEC08000000000000000000000000001137581F4*7F
$QZQSM,57,53ADC36D350006D350D1FD8DEC08000000000000000000000000001137581F4*7A
$QZQSM,56,9AADF36D0A0002C3E8588ACB118162352C474588FCB1F4165DC0001153193F4*00
$QZQSM,61,9AADF36D0A0002C3E8588ACB118162352C474588FCB1F4165DC0001153193F4*04
$QZQSM,57,9AADF36D0A0002C3E8588ACB118162352C474588FCB1F4165DC0001153193F4*01
$QZQSM,56,C6ADF36D0A0002CC2659860B30E966222D0045A710B4FB286220001195BDFA4*0F
$QZQSM,61,C6ADF36D0A0002CC2659860B30E966222D0045A710B4FB286220001195BDFA4*0B
$QZQSM,57,C6ADF36D0A0002CC2659860B30E966222D0045A710B4FB286220001195BDFA4*0E
$QZQSM,56,53ADF36D0A00051004A2710000000000000000000000000000000011EA62D30*00
$QZQSM,61,53ADF36D0A00051004A2710000000000000000000000000000000011EA62D30*04
$QZQSM,57,53ADF36D0A00051004A2710000000000000000000000000000000011EA62D30*01
$QZQSM,56,9AADC36D350006D350D1FD8DEC08000000000000000000000000001226C6A8C*07
$QZQSM,61,9AADC36D350006D350D1FD8DEC08000000000000000000000000001226C6A8C*03
$QZQSM,57,9AADC36D350006D350D1FD8DEC08000000000000000000000000001226C6A8C*06
$QZQSM,56,C6ADF36D0A0002C3E8588ACB118162352C474588FCB1F4165DC000124B5BA8C*7A
$QZQSM,61,C6ADF36D0A0002C3E8588ACB118162352C474588FCB1F4165DC000124B5BA8C*7E
$QZQSM,57,C6ADF36D0A0002C3E8588ACB118162352C474588FCB1F4165DC000124B5BA8C*7B
$QZQSM,56,53ADF36D0A0002CC2659860B30E966222D0045A710B4FB286220001290F82C4*74
$QZQSM,61,53ADF36D0A0002CC2659860B30E966222D0045A710B4FB286220001290F82C4*70
$QZQSM,57,53ADF36D0A0002CC2659860B30E966222D0045A710B4FB286220001290F82C4*75
$QZQSM,56,9AADF36D0A00051004A2710000000000000000000000000000000012FBFC648*01
$QZQSM,61,9AADF36D0A00051004A2710000000000000000000000000000000012FBFC648*05
$QZQSM,57,9AADF36D0A00051004A2710000000000000000000000000000000012FBFC648*00
$QZQSM,56,C6ADC36D350006D350D1FD8DEC08000000000000000000000000001316E4710*00
$QZQSM,61,C6ADC36D350006D350D1FD8DEC08000000000000000000000000001316E4710*04
$QZQSM,57,C6ADC36D350006D350D1FD8DEC08000000000000000000000000001316E4710*01
$QZQSM,56,53ADF36D0A0002C3E8588ACB118162352C474588FCB1F4165DC00013667E308*7A
$QZQSM,61,53ADF36D0A0002C3E8588ACB118162352C474588FCB1F4165DC00013667E308*7E
$QZQSM,57,53ADF36D0A0002C3E8588ACB118162352C474588FCB1F4165DC00013667E308*7B
$QZQSM,56,9AADF36D0A0002CC2659860B30E966222D0045A710B4FB2862200013A906D58*0E
$QZQSM,61,9AADF36D0A0002CC2659860B30E966222D0045A710B4FB2862200013A906D58*0A
$QZQSM,57,9AADF36D0A0002CC2659860B30E966222D0045A710B4FB2862200013A906D58*0F
$QZQSM,56,C6ADF36D0A00051004A2710000000000000000000000000000000013CBDEBD4*04
$QZQSM,61,C6ADF36D0A00051004A2710000000000000000000000000000000013CBDEBD4*00
$QZQSM,57,C6ADF36D0A00051004A2710000000000000000000000000000000013CBDEBD4*05
```

## データを読み解くには
python製のDCR DecoderのAzarashiなどを用いて、QZQSMをデコードすると、以下のような情報が出力されます。

```
防災気象情報(火山)(発表)(通常)
火山に関連する情報をお知らせします。

発表時刻: 6月27日18時42分

火山名: 口永良部島
日時: 6月27日9時42分
現象: レベル3(入山規制)

鹿児島県屋久島町

防災気象情報(海上)(発表)(通常)
海上警報が発表されました。

発表時刻: 6月27日17時20分

警報等情報要素: 海上風警報
日本海西部

警報等情報要素: 海上風警報
対馬海峡
```