def set_output_register_bit(address, val):
    """
    It is used to export values to the Output Bit General Purpose Register area of the Industrial Ethernet(EtherNet/IP, PROFINET) Slave.

    :param address: Address value of Output Bit GPR area in Industrial Ethernet Slave(0-63).
    :param val: ON : 1, OFF : 0
    :return: 0 - Success, Negative value - Failure
    """
    return 0


def set_output_register_int(address, val):
    """
    It is used to export values to the Output Int General Purpose Register area of the Industrial Ethernet(EtherNet/IP, PROFINET) Slave.

    :param address: Address value of Output Int GPR area in Industrial Ethernet Slave(0-23)
    :param val: Int value(4bytes)
    :return: 0 - Success, Negative value - Failure
    """
    return 0


def set_output_register_float(address, val):
    """
    It is used to export values to the Output Float General Purpose Register area of the Industrial Ethernet(EtherNet/IP, PROFINET) Slave.

    :param address: Address value of Output Int GPR area in Industrial Ethernet Slave(0-23)
    :param val: Float value(4bytes)
    :return: 0 - Success, Negative value - Failure
    """
    return 0


def get_output_register_bit(address):
    """
    It is used to import values to the Output Bit General Purpose Register area of the Industrial Ethernet(EtherNet/IP, PROFINET) Slave.

    :param address: Address value of Output Bit GPR area in Industrial Ethernet Slave(0-63).
    :return: Corresponding register value
    """
    return 0


def get_output_register_int(address):
    """
    It is used to import values to the Output Int General Purpose Register area of the Industrial Ethernet(EtherNet/IP, PROFINET) Slave.

    :param address: Address value of Output Int GPR area in Industrial Ethernet Slave(0-23).
    :return: Corresponding register value.
    """
    return 0


def get_output_register_float(address):
    """
    It is used to import values to the Output Float General Purpose Register area of the Industrial Ethernet(EtherNet/IP, PROFINET) Slave.

    :param address: Address value of Output Float GPR area in Industrial Ethernet Slave(0-23).
    :return: Corresponding register value.
    """
    return 0


def get_input_register_bit(address):
    """
    It is used to import values to the Input Bit General Purpose Register area of the Industrial Ethernet(EtherNet/IP, PROFINET) Slave.

    :param address: Address value of Input Bit GPR area in Industrial Ethernet Slave(0-63).
    :return: Corresponding register value
    """
    return 0


def get_input_register_int(address):
    """
    It is used to import values to the Input Int General Purpose Register area of the Industrial Ethernet(EtherNet/IP, PROFINET) Slave.

    :param address: Address value of Input Int GPR area in Industrial Ethernet Slave(0-23).
    :return: Corresponding register value.
    """
    return 0


def get_input_register_float(address):
    """
    It is used to import values to the Input Float General Purpose Register area of the Industrial Ethernet(EtherNet/IP, PROFINET) Slave.

    :param address: Address value of Input Float GPR area in Industrial Ethernet Slave(0-23).
    :return: Corresponding register value.
    """
    return 0
