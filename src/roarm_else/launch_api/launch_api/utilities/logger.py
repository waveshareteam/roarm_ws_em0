import launch.logging


def logging_screen_handler_remover(logger):
    screen_handler = launch.logging.launch_config.get_screen_handler()

    if screen_handler in logger.handlers:
        logger.removeHandler(screen_handler)


def logging_logfile_handler_remover(logger):
    file_handler = launch.logging.launch_config.get_log_file_handler()

    if file_handler in logger.handlers:
        logger.removeHandler(file_handler)
        