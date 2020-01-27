function setup_cycle_through_displays() {
    $("#top").html(
        '<input type="button" value="Fade in next" id="my-button"></input>'
    )
    var displays = [
        function() {
            show_black_screen(
                function() {
                    console.log("Black screen clicked")
                }
            )
        },
        function() {
            multiple_choice_prompt(
                "How are you?", ["good", "okay", "bad"],
                _log_value
            )
        },
        function() {
            make_text_entry(
                "What's your name?",
                "Done",
                _log_value
            )
        }
    ];
    var idx = 0
        //displays[idx]();
    $("#my-button").click(function() {
        var num_displays = displays.length;
        idx = (idx + 1) % displays.length;
        displays[idx]();
    })
}

function show_black_screen(callback_fn) {
    var parent_selector = "#black";

    // remove other callbacks that have been added
    $(parent_selector).unbind();

    _show_element_and_hide_siblings(parent_selector)
    $(parent_selector).click(function() {

        if (typeof callback_fn !== "undefined") {
            callback_fn();
        }

        $.when($(parent_selector).fadeOut())
    })
}

function make_text_entry(content, button, callback_fn) {

    var parent_selector = "#col-1"
    var content_selector = "#col-1-content"

    var display_html = ''
    display_html += _prepare_content(content)
    display_html += '<input type="text"> <br>'
    display_html += _make_buttons(button, false)

    $(content_selector).html(display_html)

    _prompt(
        parent_selector,
        content_selector,
        _get_text_value,
        content_selector,
        callback_fn,
        content_selector
    );
}

function multiple_choice_prompt(content, buttons, callback_fn) {

    _two_col_prompt(
        content,
        _make_buttons(buttons),
        _get_pushed_button_value,
        undefined,
        callback_fn
    );
}

function _two_col_prompt(
    content,
    input,
    get_value_fn,
    selector_to_element_of_interest,
    callback_fn
) {

    var parent_selector = "#col-2";
    var content_selector = "#col-2-left";
    var input_selector = "#col-2-right";

    $(content_selector).html(_prepare_content(content));
    $(input_selector).html(input);

    var selectors_to_clear = [content_selector, input_selector]
    _prompt(
        parent_selector,
        input_selector,
        get_value_fn,
        selector_to_element_of_interest,
        callback_fn,
        selectors_to_clear
    );
}

/* 
 * Inputs:
 *   parent_selector: 
        The selector that should be shown for this prompt and then
        faded away after the prompt has been answered.
 *   input_selector: 
        The selector that has a button as a child that should be 
        used as submit, note that other buttons can be in other 
        places, e.g., a div in the parent or even a div in the 
        input selector.
 *   get_value_fn: 
        Get a value after a button has been clicked, the function 
        takes 'this' (the button clicked) and the selector of 
        interest as arguements.  The selector of interest can be 
        used to grab the relevant value, such as in a text field
        or slider.
 *   callback_fn:
 *      The callback function takes the value returned by 
        get_value_fn and does something with it, e.g., publishes
        it to a ROS network.
 *   selectors_to_clear:
        These are selectors (array or single string) that will 
        have their HTML field set to an empty string (""). This
        helps make sure that things are cleaned up after they are
        used and that previous questions are not popped up before
        new content is loaded. 
 */
function _prompt(
    parent_selector,
    input_selector,
    get_value_fn,
    selector_to_element_of_interest,
    callback_fn,
    selectors_to_clear
) {

    _show_element_and_hide_siblings(parent_selector)

    $(input_selector).children("input:button").click(function() {

        var value = get_value_fn(this, selector_to_element_of_interest)
        if (typeof callback_fn !== "undefined") {
            callback_fn(value);
        }

        $.when($(parent_selector).fadeOut())
            .done(
                function() {
                    _clear_html_elements(selectors_to_clear)
                }
            );
    })
}

function _show_element_and_hide_siblings(selector_str) {
    var selector = $(selector_str);
    selector.fadeIn("slow");
    selector.siblings().hide();
}

function _prepare_content(content) {
    return '<p>' + content + '</p>';
}

function _make_buttons(buttons, is_each_button_on_new_line = true) {
    buttons = _make_sure_is_array(buttons)
    out = ''
    for (var i = 0; i < buttons.length; i++) {
        out += '<input type="button" value="' + buttons[i] + '" />';

        // put a break in between buttons and not after the last button
        if (is_each_button_on_new_line && i < buttons.length - 1) {
            out += '<br>'
        }
    }
    return out
}

function _clear_html_elements(selectors) {
    selectors = _make_sure_is_array(selectors)
    for (var i = 0; i < selectors.length; i++) {
        $(selectors[i]).html("")
    }

}

function _make_sure_is_array(value) {
    if (!Array.isArray(value)) {
        value = [value]
    }
    return value
}

function _get_pushed_button_value(button, _) {
    return button.value
}

function _get_text_value(_, parent_selector) {
    var text_inputs = $(parent_selector).children("input:text")
    if (text_inputs.length != 1) {
        throw "There should only be one text entry element"
    }
    return text_inputs[0].value
}

function _log_value(value) {
    console.log(value)
}