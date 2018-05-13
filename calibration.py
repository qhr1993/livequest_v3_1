def calibration(mode,vref):
    # mode => 0 adc calibration
    # mode => 1 sensor calibration
    import peripheral_query
    import time
    import ujson
    import uio
    from machine import Pin
    from peripheral_query import GasData

    print('[4]===================')

    calibration_frequency = 20 # seconds
    calibration_times = 100

    calibration_NH3 = []
    calibration_SO2 = []
    calibration_H2S = []

    iter_calibration = 0

    while iter_calibration < calibration_times:
        adc0 = ADC(id=0)

        outer_iter = 0
        outer_iter_times = 20
        outer_buff_NH3 = []
        outer_buff_SO2 = []
        outer_buff_NH3 = []

        while outer_iter < outer_iter_times:
            filtered_mvolts = _get_filtered_mvolts(adc0,vref)
            outer_buff_NH3.append(filtered_mvolts.NH3)
            outer_buff_SO2.append(filtered_mvolts.SO2)
            outer_buff_H2S.append(filtered_mvolts.H2S)
            outer_iter = outer_iter + 1
        buff_nh3 = sum(outer_buff_NH3)/outer_iter_times
        buff_so2 = sum(outer_buff_SO2)/outer_iter_times
        buff_h2s = sum(outer_buff_H2S)/outer_iter_times
        print('[4]sample #: ' + ('%d' % (iter_calibration + 1)) + ' / ' + ('%d' % (calibration_times + 1)))
        print('[4]sample NH3: ' + ('%.1f' % (buff_nh3)))
        print('[4]sample SO2: ' + ('%.1f' % (buff_so2)))
        print('[4]sample H2S: ' + ('%.1f' % (buff_h2s)))

        calibration_NH3.append(buff_nh3)
        calibration_SO2.append(buff_so2)
        calibration_H2S.append(buff_h2s)

        adc0.deinit()
        iter_calibration = iter_calibration + 1
        time.sleep(calibration_frequency)

    cfg_nh3 = sum(calibration_nh3)/calibration_times
    cfg_so2 = sum(calibration_so2)/calibration_times
    cfg_h2s = sum(calibration_h2s)/calibration_times

    with uio.open('/flash/configure.json', 'r', encoding = "utf-8") as handle:
        psd_json = ujson.load(handle)
    if mode == 0:
        psd_json["calibration"]["sensor_nh3"]["bias"] = round(cfg_nh3,1)
        psd_json["calibration"]["sensor_so2"]["bias"] = round(cfg_so2,1)
        psd_json["calibration"]["sensor_h2s"]["bias"] = round(cfg_h2s,1)
        print('[4]Calibration finished...')
        print('[4]NH3 bias: '+ ('%.1f' % (cfg_nh3)))
        print('[4]SO2 bias: '+ ('%.1f' % (cfg_so2)))
        print('[4]H2S bias: '+ ('%.1f' % (cfg_h2s)))
    elif mode == 1:
        psd_json["calibration"]["sensor_nh3"]["i0"] = round((cfg_nh3 - psd_json["calibration"]["sensor_nh3"]["bias"])/624,1)
        psd_json["calibration"]["sensor_so2"]["i0"] = round((cfg_so2 - psd_json["calibration"]["sensor_so2"]["bias"])/624,1)
        psd_json["calibration"]["sensor_h2s"]["i0"] = round((cfg_h2s - psd_json["calibration"]["sensor_h2s"]["bias"])/624,1)
        print('[4]Calibration finished...')
        print('[4]NH3 zero-drift: '+ ('%.1f' % ((cfg_nh3 - psd_json["calibration"]["sensor_nh3"]["bias"])/624)))
        print('[4]SO2 zero-drift: '+ ('%.1f' % ((cfg_so2 - psd_json["calibration"]["sensor_so2"]["bias"])/624)))
        print('[4]H2S zero-drift: '+ ('%.1f' % ((cfg_h2s - psd_json["calibration"]["sensor_h2s"]["bias"])/624)))

    with uio.open('/flash/configure.json', 'w', encoding = "utf-8") as handle:
        ujson.dumps(psd_json, handle)
        print('[4]Configure file written...)
