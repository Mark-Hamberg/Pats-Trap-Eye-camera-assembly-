def pickit_connect(ip):
    """
    This function establishes communication with the vision system.
    The default IP of PickIt is 192.168.66.1, and the user uses PickIt IP in the same band as Robot IP.
    See the Pickit Support site for details on how to use it.

    :param ip: Server IP of Pickit 3D (ex, 192.168.137.90)
    :return: 0 Connection success, -1 Connection failed.
    """
    return 0


def pickit_disconnect():
    """
    This function terminates the connection to the vision system.

    """
    return None


def pickit_request_calibration():
    """
    This function requests a calibration once from the vision system.

    :return: 0 Connection success, -1 Connection failed.
    """
    return 0


def pickit_change_configuration(setup_id, product_id):
    """
    This function loads setup_id and product_id set in the vision system.

    :param setup_id: int - The setup_id number stored in the Pickit server.
    :param product_id: int - The product_id number stored in the Pickit server
    :return: 0 Connection success, -1 Connection failed.
    """
    return 0


def pickit_save_snapshot():
    """
    This function saves the snapshot to the server.
    """
    return None


def pickit_detection(offset_z):
    """
    This function detects the input model and returns (pick_ pick_pos

    :param offset_z: The offset_z sets to 'pick_prepos' distance.
    :return: Return Value Data type Description:
        data_dictionary = {'pick_pos':pick_pos, 'pick_prepos':pick_prepos, 'object_age':data['object_age'],
        'object_type':data['object_type'], 'object_dimensions':data['object_dimensions'],
        'object_remaining':data['objects_remaining'], 'status':data['status']}
    """
    return None


def pickit_next_object(offset_z):
    """
    This function return s pick_prepos and pick_pos detected next to the input model.

    :param offset_z: The offset_z sets to 'pick_prepos' distance.
    :return: Return Value Data type Description:
        data_dictionary = {'pick_pos':pick_pos, 'pick_prepos':pick_prepos, 'object_age':data['object_age'],
        'object_type':data['object_type'], 'object_dimensions':data['object_dimensions'],
        'object_remaining':data['objects_remaining'], 'status':data['status']}
    """
    return None