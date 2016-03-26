var group__packet =
[
    [ "command_packet_t", "group__packet.html#structcommand__packet__t", [
      [ "commandTag", "group__packet.html#a78f4d8266c50f261837805eb8596de59", null ],
      [ "flags", "group__packet.html#a0f424c2b52c19c32bb8d5da2a2a474ac", null ],
      [ "parameterCount", "group__packet.html#ad17517a7234031a3bab34ae43f44d613", null ],
      [ "reserved", "group__packet.html#a010bf97f924783aa47d8111da19eb155", null ]
    ] ],
    [ "flash_erase_all_packet_t", "group__packet.html#structflash__erase__all__packet__t", [
      [ "commandPacket", "group__packet.html#a37826352c73d7b8c1d9119340a937fa3", null ],
      [ "memoryId", "group__packet.html#a20221dfe400a50e448cf94838b2df75f", null ]
    ] ],
    [ "flash_erase_region_packet_t", "group__packet.html#structflash__erase__region__packet__t", [
      [ "byteCount", "group__packet.html#aef5f0e1fa7b63f3c7795ec0d20425c2c", null ],
      [ "commandPacket", "group__packet.html#a3698ffeba5129ba07ca50d9ad2edef31", null ],
      [ "startAddress", "group__packet.html#a9dfbc3333b2e19554bd60c417b57513d", null ]
    ] ],
    [ "get_property_packet_t", "group__packet.html#structget__property__packet__t", [
      [ "commandPacket", "group__packet.html#a6c6cb1ab0d78e08ad14bd23c554a3e21", null ],
      [ "memoryId", "group__packet.html#aac7989d77b0e941ac3f450e3ae3c4867", null ],
      [ "propertyTag", "group__packet.html#a22a4cd19aae535958ec3204e93d41a5d", null ]
    ] ],
    [ "set_property_packet_t", "group__packet.html#structset__property__packet__t", [
      [ "commandPacket", "group__packet.html#a1f8df0c080a65798bc53a6ed967a1cc1", null ],
      [ "propertyTag", "group__packet.html#a78dc9bb1086f745ef8837a4558679e9e", null ],
      [ "propertyValue", "group__packet.html#a138fd3c555ed30d1faaedb7dc9f6aa7c", null ]
    ] ],
    [ "receive_sb_file_packet_t", "group__packet.html#structreceive__sb__file__packet__t", [
      [ "byteCount", "group__packet.html#aa6e31d8c68afd8fb504150cc551af134", null ],
      [ "commandPacket", "group__packet.html#a0005fe0d4c037552472d33d56dd3b554", null ]
    ] ],
    [ "write_memory_packet_t", "group__packet.html#structwrite__memory__packet__t", [
      [ "byteCount", "group__packet.html#aee73997ea56cf6efbff5605ba1746b97", null ],
      [ "commandPacket", "group__packet.html#a52f3cad3c7a621eaec8a5c274bc17bbe", null ],
      [ "startAddress", "group__packet.html#ab9d393e2c299178aac696be5000d988b", null ]
    ] ],
    [ "read_memory_packet_t", "group__packet.html#structread__memory__packet__t", [
      [ "byteCount", "group__packet.html#a64f35c65ae56a21f24ce915f7682a649", null ],
      [ "commandPacket", "group__packet.html#a26adcdf054bd4778df2634ccf6e0c970", null ],
      [ "startAddress", "group__packet.html#a1cd18144f5eedd7c0c02addc4552f3a3", null ]
    ] ],
    [ "fill_memory_packet_t", "group__packet.html#structfill__memory__packet__t", [
      [ "byteCount", "group__packet.html#a6fb6d466126eee41b190b19033b3b023", null ],
      [ "commandPacket", "group__packet.html#abd410dfda1d41d6355e90fd17ccd2eb3", null ],
      [ "patternWord", "group__packet.html#a061fc97cbd4374fb9f576e6ceb297b67", null ],
      [ "startAddress", "group__packet.html#a15cb95e2f9c841b8033c6c93f3a47dd4", null ]
    ] ],
    [ "execute_call_packet_t", "group__packet.html#structexecute__call__packet__t", [
      [ "argumentWord", "group__packet.html#a59d5c127039a03e786dc5744b8d87410", null ],
      [ "callAddress", "group__packet.html#aa4203770b1cf902d210bc39581c98197", null ],
      [ "commandPacket", "group__packet.html#acc3d949a150d1e41cb8bcf68764a619f", null ],
      [ "stackpointer", "group__packet.html#a74c35a297c96aeae6fe495477dcb6af3", null ]
    ] ],
    [ "flash_security_disable_packet_t", "group__packet.html#structflash__security__disable__packet__t", [
      [ "commandPacket", "group__packet.html#a82f3df4f8664297a7b11997b50a6e195", null ],
      [ "keyHigh", "group__packet.html#a7ce5ac5e2faad2fbca49456fac9c3763", null ],
      [ "keyLow", "group__packet.html#a3976cfeba7bd09708a1e5df4e055ce9f", null ]
    ] ],
    [ "flash_program_once_packet_t", "group__packet.html#structflash__program__once__packet__t", [
      [ "byteCount", "group__packet.html#ab65ae6e00250e1e5f16ce1dbd55f5e3e", null ],
      [ "commandPacket", "group__packet.html#a20be93281f125b289b37c48b94b18c17", null ],
      [ "data", "group__packet.html#aa0c5e372ceaa51d983b40539720cb133", null ],
      [ "index", "group__packet.html#aef9847bbe4d9fc94f4e040271e7252b5", null ]
    ] ],
    [ "flash_read_once_packet_t", "group__packet.html#structflash__read__once__packet__t", [
      [ "byteCount", "group__packet.html#a94c6cafb63782713b783d96295aee37e", null ],
      [ "commandPacket", "group__packet.html#ac57d92fae78b88d257ecbc66447f9346", null ],
      [ "index", "group__packet.html#aa96051dd46225c1a1178f1cebd5a4617", null ]
    ] ],
    [ "flash_read_resource_packet_t", "group__packet.html#structflash__read__resource__packet__t", [
      [ "byteCount", "group__packet.html#a20751de00bb58d6d7c6e0c467cb2b750", null ],
      [ "commandPacket", "group__packet.html#ad4f00cd55380f248b799907194898c73", null ],
      [ "option", "group__packet.html#ab0c742ccfc347e29c1a2f8f58931412a", null ],
      [ "startAddress", "group__packet.html#a922b662a97c2fa72d59fc794c69bb5cc", null ]
    ] ],
    [ "configure_quadspi_packet_t", "group__packet.html#structconfigure__quadspi__packet__t", [
      [ "commandPacket", "group__packet.html#a9f57e4eb2b7ea0e084cd60f7d0bf1636", null ],
      [ "configBlockAddress", "group__packet.html#af30f187b349e33d3c45274514c6e8402", null ],
      [ "flashMemId", "group__packet.html#ab8a7b4e597ca09d7b1be116d14a12408", null ]
    ] ],
    [ "generic_response_packet_t", "group__packet.html#structgeneric__response__packet__t", [
      [ "commandPacket", "group__packet.html#a7926a8d067ec6c1bcd950aab4b8c64e5", null ],
      [ "commandTag", "group__packet.html#ad8260ed863b7968460694e8a415a9588", null ],
      [ "status", "group__packet.html#a2bca0868fe6f02d5deb0ed75b1d7fef3", null ]
    ] ],
    [ "get_property_response_packet_t", "group__packet.html#structget__property__response__packet__t", [
      [ "commandPacket", "group__packet.html#a6b2fed0f4cd95a3f3b9c3479053feebc", null ],
      [ "propertyValue", "group__packet.html#a8baabd5441b058bc984b3fee721be304", null ],
      [ "status", "group__packet.html#a2fd6157a93e015b9744fee7eb0e94b80", null ]
    ] ],
    [ "read_memory_response_packet_t", "group__packet.html#structread__memory__response__packet__t", [
      [ "commandPacket", "group__packet.html#ace46643f9859d0d6e400f2a19a4369db", null ],
      [ "dataByteCount", "group__packet.html#a04969786a1c8c692a9371f44a4d2298f", null ],
      [ "status", "group__packet.html#a299514d80e44f999be3d2726f4396536", null ]
    ] ],
    [ "flash_read_once_response_packet_t", "group__packet.html#structflash__read__once__response__packet__t", [
      [ "byteCount", "group__packet.html#a58c05fa51ecc291efdbd0f2d94aa00bb", null ],
      [ "commandPacket", "group__packet.html#a33d7be4cdd5a10484f857af8a7169610", null ],
      [ "data", "group__packet.html#a5121eae249f8dc0a66a672d88f0a1080", null ],
      [ "status", "group__packet.html#a119596593d415747fc983ea8a0ca9ca0", null ]
    ] ],
    [ "flash_read_resource_response_packet_t", "group__packet.html#structflash__read__resource__response__packet__t", [
      [ "commandPacket", "group__packet.html#abb930d955f76fb01475420d84defbe68", null ],
      [ "dataByteCount", "group__packet.html#a4dce9ecb14e42e4473ff264cb951645f", null ],
      [ "status", "group__packet.html#a39093106bdfd5c7c85749ea21df9c4f6", null ]
    ] ],
    [ "framing_header_t", "group__packet.html#structframing__header__t", [
      [ "packetType", "group__packet.html#a469b3b969cd29cad932b5852a300f66e", null ],
      [ "startByte", "group__packet.html#ac2a9a5d32eb4c25e505f3985cabf82e9", null ]
    ] ],
    [ "framing_sync_packet_t", "group__packet.html#structframing__sync__packet__t", [
      [ "header", "group__packet.html#af1e5f7ed5b40e2d837160964b16f109f", null ]
    ] ],
    [ "framing_data_packet_t", "group__packet.html#structframing__data__packet__t", [
      [ "crc16", "group__packet.html#a1df73108dfcd9c036d9e90745c8c5b01", null ],
      [ "header", "group__packet.html#a4097c9e4c228ee2229e33bd0df3a1886", null ],
      [ "length", "group__packet.html#a756d79627986699ace69256a3f2b2fda", null ]
    ] ],
    [ "serial_framing_packet_t", "group__packet.html#structserial__framing__packet__t", [
      [ "data", "group__packet.html#a1663ead9ade4914ca366f32eb700b2d4", null ],
      [ "dataPacket", "group__packet.html#ab5ec4afa89810a31aeccb51b8b9e2857", null ]
    ] ],
    [ "serial_data_t", "group__packet.html#structserial__data__t", [
      [ "callbackBuffer", "group__packet.html#aeb271e952915940873ca56f3fd9e9836", null ],
      [ "data", "group__packet.html#aba1517a11e5b30c36111e135528b5561", null ],
      [ "framingPacket", "group__packet.html#af554530489754635d44988012a9f765e", null ],
      [ "isAckAbortNeeded", "group__packet.html#a0cd993c77e21580fb5a431cebcf120aa", null ],
      [ "isAckNeeded", "group__packet.html#add947cb22765d9423dd0b7ece37e4dd1", null ],
      [ "isBackToBackWrite", "group__packet.html#aff919b95907cf3794c6926d4c60ec749", null ],
      [ "readOffset", "group__packet.html#aade960084a345f615a2bf293bed8db6c", null ],
      [ "writeOffset", "group__packet.html#a6b3b8437f398c7a03f8169111580524d", null ]
    ] ],
    [ "ping_response_t", "group__packet.html#structping__response__t", [
      [ "crc16", "group__packet.html#a83bf50d480817376c00c27ce6ea0c517", null ],
      [ "options", "group__packet.html#a70fb299b62641852b8a7770de78ab46c", null ],
      [ "version", "group__packet.html#a2b556418beb11d0d32e9738f3450083a", null ]
    ] ],
    [ "call_function_t", "group__packet.html#ga258d32915aaa6e17e506dcef25a450ea", null ],
    [ "_command_packet_constants", "group__packet.html#gad3df2b7984d0e28379a9a42feb27a873", [
      [ "kMaxPropertyReturnValues", "group__packet.html#ggad3df2b7984d0e28379a9a42feb27a873ad0804f2c4956f6fbb33e072703e109cd", null ],
      [ "kMaxProgramOnceValues", "group__packet.html#ggad3df2b7984d0e28379a9a42feb27a873a93eb60fa3641a4a11cbfb6be2b35772f", null ],
      [ "kCommandTagCount", "group__packet.html#ggad3df2b7984d0e28379a9a42feb27a873a565f0323e8b6751f6ca47a6d128f876b", null ]
    ] ],
    [ "_command_packet_flags", "group__packet.html#gaf117778d1938807c8e0fffeb39c672df", null ],
    [ "_command_tags", "group__packet.html#gaefc521e8d8a1bb32e9278d8f92f1115e", [
      [ "kLastCommandTag", "group__packet.html#ggaefc521e8d8a1bb32e9278d8f92f1115ead37204f9baf3f14abcefc9d8d82d88ea", null ],
      [ "kResponseCommandHighNibbleMask", "group__packet.html#ggaefc521e8d8a1bb32e9278d8f92f1115ead61de9bf3e638999f380e8bf4b3f3d67", null ]
    ] ],
    [ "_flash_mem_id", "group__packet.html#ga3b7b39313fa770abac072b813fbe2e60", null ],
    [ "_framing_packet_constants", "group__packet.html#ga62b42db1f8bbcdcd1a9e28220ee4627e", null ],
    [ "_serial_packet_constants", "group__packet.html#gac4680247a99210ad5bb3e2dfb634dfd3", null ],
    [ "_serial_packet_mode", "group__packet.html#ga2a8f13fe838021db8eeaf217ff17a9db", null ],
    [ "_serial_protocol_version_constants", "group__packet.html#ga012b8bb615153c433c06843e404084b0", null ],
    [ "_timing_constants", "group__packet.html#ga2c66f7e3d2dacb6bf4f217f66e60ed00", [
      [ "kDefaultByteReadTimeoutMs", "group__packet.html#gga2c66f7e3d2dacb6bf4f217f66e60ed00ad24537294d9e3209afa42a1ee7081f4d", null ],
      [ "kCallbackBufferSize", "group__packet.html#gga2c66f7e3d2dacb6bf4f217f66e60ed00a54286149a8cc228b9668daf91c21dfda", null ]
    ] ],
    [ "calculate_framing_crc16", "group__packet.html#ga218748fd1c0987305a058ad4f24bbe95", null ],
    [ "read_crc16", "group__packet.html#ga78781b6dc665ce57e4ff3f5b1c9fc8f0", null ],
    [ "read_data", "group__packet.html#gabeb657f00ab168f281a435ede2232e67", null ],
    [ "read_data_packet", "group__packet.html#gab27002b66fbf490c4975ae374608303d", null ],
    [ "read_header", "group__packet.html#gaf00c2f837635865d4d1059aeca00d6f2", null ],
    [ "read_length", "group__packet.html#ga9c52fd15222fe89c53e0f16809e99051", null ],
    [ "read_start_byte", "group__packet.html#ga30d2176721f80537c2a242c1886243d3", null ],
    [ "send_deferred_ack", "group__packet.html#ga4117d35f00b344603bb673f75832d9a0", null ],
    [ "serial_packet_abort", "group__packet.html#ga1a60f41326f758000304976fd18d65a2", null ],
    [ "serial_packet_finalize", "group__packet.html#ga68e805a1882eb220f1ddd9a08c26030b", null ],
    [ "serial_packet_get_max_packet_size", "group__packet.html#ga598a287b57672291406cff9c728e03f9", null ],
    [ "serial_packet_init", "group__packet.html#ga3707af76120e3f97302536c0615f7add", null ],
    [ "serial_packet_queue_byte", "group__packet.html#ga88407143978559f83d7115b86dc876be", null ],
    [ "serial_packet_read", "group__packet.html#gaec8900ed4ea5a3c981d66dab38516643", null ],
    [ "serial_packet_send_sync", "group__packet.html#gab0420ede400b98b1c21a7a7713b57633", null ],
    [ "serial_packet_write", "group__packet.html#ga7bcf4e62f6cbc2ab39c1666c1d5e752a", null ],
    [ "serial_send_ping_response", "group__packet.html#gabb5327f7eac946ecd5bf73b12565a502", null ],
    [ "wait_for_ack_packet", "group__packet.html#ga34f308d460efca1f58e045f8ee3f09de", null ],
    [ "write_data", "group__packet.html#ga133c8f47194b9aec1935136a09ff0a69", null ],
    [ "g_serialContext", "group__packet.html#ga625bcffc6e8f34be003eac627b5414dc", null ],
    [ "k_PingResponse", "group__packet.html#gadd274c1ccd09e20f83594c95076d1218", null ]
];