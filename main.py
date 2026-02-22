import asyncio
from logger import setup_print_logger
from controller import FlightController

if __name__ == "__main__":
    log_path = setup_print_logger(log_dir="logs", prefix="flight")
    print(f"[LOG] Writing console output to {log_path}")
    asyncio.run(FlightController().run())
