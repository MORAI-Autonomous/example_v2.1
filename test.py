import dearpygui.dearpygui as dpg

dpg.create_context()
dpg.create_viewport(width=800, height=500)
dpg.setup_dearpygui()

PADDING   = 60
LOG_H     = 100
LOG_BAR_H = 0   # log 툴바 없으므로 0

def get_top_h():
    return dpg.get_viewport_height() - PADDING - LOG_H

def get_w():
    return dpg.get_viewport_width()

def get_log_inner_h():
    return LOG_H - 8   # border 여백 제외

def get_log_inner_w():
    return get_w()  # border + scrollbar 여백 제외

with dpg.window(tag="main", no_title_bar=True, no_scrollbar=True):
    #with dpg.group():
    with dpg.group(horizontal=True):
        with dpg.child_window(tag="left", width=300 - 100, height=get_top_h(), border=True):
            for i in range(60):
                dpg.add_text(f"cmd line {i}")

        with dpg.child_window(tag="right", width=-1, height=get_top_h(), border=True):
            with dpg.tab_bar():
                with dpg.tab(label="Vehicle Info"):
                    for i in range(60):
                        dpg.add_text(f"monitor line {i}")

    with dpg.child_window(tag="log", width=get_log_inner_w(), height=LOG_H,
                            border=True, no_scrollbar=False):
        with dpg.child_window(tag="log_inner", width=get_log_inner_w() - 16,
                                height=get_log_inner_h(), border=False):
            for i in range(40):
                dpg.add_text(f"log line {i}")

def _on_resize(s, a):
    top_h = get_top_h()
    w     = get_w()
    liw   = get_log_inner_w()
    lih   = get_log_inner_h()
    dpg.configure_item("left",      height=top_h)
    dpg.configure_item("right",     width=w - 316, height=top_h)
    dpg.configure_item("log",       width=liw)
    dpg.configure_item("log_inner", width=liw - 16, height=lih)
    dpg.set_item_width("main",  w)
    dpg.set_item_height("main", dpg.get_viewport_height())

dpg.set_viewport_resize_callback(_on_resize)
dpg.set_primary_window("main", True)
dpg.show_viewport()

while dpg.is_dearpygui_running():
    dpg.render_dearpygui_frame()
dpg.destroy_context()