use std::{sync::mpsc, time::Duration};

const COM_PORT:&str = "COM11";
const BAUD_RATE:u32 = 9600;

fn main() {
    let mut port = serialport::new(COM_PORT, BAUD_RATE)
        .timeout(Duration::from_millis(10))
        .open()
        .expect("Failed to open serial port");

    let (tx, rx) = mpsc::channel();

    // 揮発設定を実施
    let l1s_enable = [0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x20, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x03, 0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x05, 0x05, 0x06, 0x08, 0x0E, 0x00, 0x01, 0x00, 0x01, 0x01, 0x59, 0x57];
    let sfrbx_enable = [0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x13, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x25, 0x3D];
    let _ = port.write_all(l1s_enable.as_slice());
    let _ = port.write_all(sfrbx_enable.as_slice());

    // 受信スレッド
    std::thread::spawn(move || loop {
        loop {
            // シリアルポートから受信
            let mut buf: Vec<u8> = vec![0u8; 1024];
            let len = port.read(buf.as_mut_slice());
            if let Ok(len) = len {
                if len > 0 {
                    // 1バイト単位に分解してチャネルに投入
                    for i in 0..len {
                        tx.send(buf[i]).unwrap();
                    }
                }
            }
        }
    });

    // メインスレッド
    loop {
        // 1バイトずつチャネルから受信
        let data = rx.recv().unwrap();

        // 解析
        if data == b'$' {
            // NMEA
            loop {
                // 0x0D 0x0A を待つ
                let data = rx.recv().unwrap();
                if data == 0x0D {
                    let data = rx.recv().unwrap();
                    if data == 0x0A {
                        break;
                    }
                }
            }
            // println!("NMEA Sentence");
        } else if data == 0xB5 {
            // SYNC1
            if rx.recv().unwrap() != 0x62 {
                continue;
            } //SYNC2
              // UBX binay
            // print!("UBX binary Sentence: ");

            let class: u8 = rx.recv().unwrap();
            let id: u8 = rx.recv().unwrap();

            let l_l = rx.recv().unwrap();
            let l_h = rx.recv().unwrap();
            let length: u16 = l_l as u16 + ((l_h as u16) << 8);

            let mut buf: Vec<u8> = vec![0u8; length as usize];
            for i in 0..length as usize {
                buf[i] = rx.recv().unwrap();
            }

            let ck_a_rcv = rx.recv().unwrap();
            let ck_b_rcv = rx.recv().unwrap();

            // チェックサム計算(微妙によくわからない計算だが仕様書通り)
            let mut ck_a: u8 = 0;
            let mut ck_b: u8 = 0;
            ck_a = ck_a.wrapping_add(class);
            ck_b = ck_b.wrapping_add(ck_a);

            ck_a = ck_a.wrapping_add(id);
            ck_b = ck_b.wrapping_add(ck_a);

            ck_a = ck_a.wrapping_add(l_l);
            ck_b = ck_b.wrapping_add(ck_a);

            ck_a = ck_a.wrapping_add(l_h);
            ck_b = ck_b.wrapping_add(ck_a);

            for i in 0..length as usize {
                ck_a = ck_a.wrapping_add(buf[i]);
                ck_b = ck_b.wrapping_add(ck_a);
            }

            if ck_a == ck_a_rcv && ck_b == ck_b_rcv {
                parse(class, id, buf);
            } else {
                println!(
                    "Checksum NG {:?},{:?} {:?},{:?} ",
                    ck_a, ck_b, ck_a_rcv, ck_b_rcv
                );
            }
        } else {
            // Invalid data
        }
    }
}

const CLASS_RXM: u8 = 0x02;
const ID_SFRBX: u8 = 0x13;

const CLASS_ACK: u8 = 0x05;
const ID_ACK: u8 = 0x01;
const ID_NAK: u8 = 0x00;

fn parse(class: u8, id: u8, payload: Vec<u8>) {
    if class == CLASS_ACK && id == ID_ACK {
        let _cls_id = payload[0];
        let _msg_id = payload[1];
        println!("ACK");
    }
    if class == CLASS_ACK && id == ID_NAK {
        let _cls_id = payload[0];
        let _msg_id = payload[1];
        println!("NAK");
    }
    if class == CLASS_RXM && id == ID_SFRBX {
        // ペイロードヘッダ
        let gnss_id = payload[0];
        let sv_id = payload[1];
        let _sig_id = payload[2]; //V2 only
        let _freq_id = payload[3];
        let num_words: u8 = payload[4];
        let _chn = payload[5]; //V2 only
        let _version: u8 = payload[6];
        let _reserved1 = payload[7];

        // ペイロードデータ本体
        let data_slice_end_index = ((num_words as usize) * 4) + 8;
        let d_wrd = &payload[8..data_slice_end_index];

        // チェック
        if gnss_id != 5{
            // println!("Not QZSS");
            return;
        }
        if num_words != 8 {
            // println!("Not num_words == 8");
            return;
        }
/*
        println!("gnss_id: {:?}",gnss_id); //QZSS 5
        println!("sv_id: {:?}",_sv_id); //QZSS 1~10
        println!("sig_id: {:?}",sig_id);
        println!("chn: {:?}",_chn);
        println!("version: {:?}",_version);
        println!("num_words: {:?}", num_words);
        println!("data_slice_end_index: {:?}", data_slice_end_index);
        println!("len: {:?}", d_wrd.len());
 */
        // svId to PRN
        let prn_l1s = match sv_id {
            1 => { 183 },
            2 => { 184 },
            3 => { 189 },
            4 => { 185 },
            5 => { 186 },
            _ => { 0 },
        };

        // PRN to Satellite ID
        let satellite_id = match prn_l1s {
            184 => {0x56},
            185 => {0x57},
            189 => {0x61},
            183 => {0x55}, // Note 1
            186 => {0x58}, // Note 2
            _ => { 0x00 },
        };

        let mut data: Vec<u8> = vec![0u8; d_wrd.len()];
        // エンディアン入れ替え
        for i in 0..(d_wrd.len()/4) {
            data[(i*4)+0] = d_wrd[(i*4)+3];
            data[(i*4)+1] = d_wrd[(i*4)+2];
            data[(i*4)+2] = d_wrd[(i*4)+1];
            data[(i*4)+3] = d_wrd[(i*4)+0];
        }

        data[31] &= 0b1100_0000; // 余剰ビットを切り捨て(256bit - 250bit = 6bitあまり)
        let data = data[..=31].to_vec();

        let mt = data[1]>>2;
        // println!("MT:{:?}", mt);

        if mt != 43 && mt !=44 {
            // println!("Not DC Report");
            return;
        }

        let mut qzqsm: String = "QZQSM,".to_owned();
        qzqsm += format!("{:02X}", satellite_id).as_str(); //サテライトID
        qzqsm += ",";
        for i in 0..(data.len()-1) {
            qzqsm += format!("{:02X}", data[i]).as_str();
        }
        qzqsm += format!("{:01X}", data[31]>>4).as_str(); //63文字である必要がある

        // NMEAチェックサム計算($～*(を含まない)間のXOR)
        let mut nmea_checksum:u8 = 0;
        for i in 0..(qzqsm.len()) {
            let c = qzqsm.as_bytes()[i];
            nmea_checksum ^= c;
        }

        qzqsm += "*";
        qzqsm += format!("{:02X}", nmea_checksum).as_str();
        qzqsm = "$".to_string() + qzqsm.as_str() + "\x0D\x0A";
        print!("{0}",qzqsm);
    }
}
