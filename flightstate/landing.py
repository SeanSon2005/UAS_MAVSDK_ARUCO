async def run_landing(controller):
    if controller.armed and not controller.land_command_sent:
        try:
            print("[FLIGHT] Landing")
            await controller.drone.action.land()
            controller.land_command_sent = True
        except Exception as exc:
            print(f"[FLIGHT] Landing command failed: {exc}")
