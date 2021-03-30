function setup_cycle_through_displays(start_idx = 0) {
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
                _log_value, [],
                2
            )
        },
        function() {
            multiple_choice_prompt_one_col(
                "How are you?", ["excellent", "good", "fair", "poor"],
                _log_value, [],
                2
            )
        },
        function() {
            text_entry_prompt(
                "What's your name?",
                "Done",
                _log_value, ['Your name'],
                3
            )
        },
        function() {
            slider_prompt(
                "How many steps today?",
                "Done",
                _log_value, ['100', '500', '50', "400"],
                3
            )
        },
        function() {
            time_entry_prompt(
                "How many steps today?",
                "Done",
                _log_value, ["5", "12:33"],
                3
            )
        },
        function() {
            numpad_prompt(
                "Enter the digits of your phone number without spaces.", ["Next"],
                _log_value, [],
                2
            )
        }
    ];
    var idx = start_idx
    displays[idx]();
    $("#my-button").click(function() {
        var num_displays = displays.length;
        idx = (idx + 1) % displays.length;
        displays[idx]();
    })
}

function show_black_screen() {
    var parent_selector = "#black";
    _show_element_and_hide_siblings(parent_selector)
}

function text_entry_prompt(
    content,
    button,
    callback_fn,
    args = [],
    seconds_before_enabling_input = 0
) {

    if (args.length > 1) {
        alert("text_entry_prompt only accepts one arg, placeholder text");
    }
    var placeholder_text = "Touch here";
    if (args.length === 1) {
        placeholder_text = args[0];
    }

    var parent_selector = "#col-1";
    var content_selector = "#col-1-content";
    var input_selector = "#col-1-input";

    var display_html = _prepare_content(content)
    var input_html = '<input type="text" id="text-input" placeholder="' + placeholder_text + '"> <br>'
    input_html += _make_buttons(button, false)

    $(content_selector).html(display_html)
    $(input_selector).html(input_html)

    _prompt(
        parent_selector,
        input_selector,
        _get_text_value,
        input_selector,
        callback_fn,
        seconds_before_enabling_input,
    );
}

function slider_prompt(
    content,
    button,
    callback_fn,
    args = [],
    seconds_before_enabling_input = 0
) {

    if (args.length !== 3 && args.length !== 4) {
        alert("slider must have three or four args: start numpad-button, end numpad-button, step_size, current_value");
    }
    var start_value = parseFloat(args[0]);
    var end_value = parseFloat(args[1]);
    var increment_value = parseFloat(args[2]) || 1;
    var current_value = parseFloat(args[3]) || (end_value - start_value) / 2;

    if (isNaN(start_value) || isNaN(end_value) || isNaN(increment_value || isNaN(current_value))) {
        alert("Args must all be floating numpad-buttons, atleast one could not be parsed");
    }

    var parent_selector = "#col-1";
    var content_selector = "#col-1-content";
    var input_selector = "#col-1-input";

    var display_html = _prepare_content(content);
    var input_html = "";
    input_html += `<p>Value: <span id="slider-value-display"></span></p>`;
    input_html += `<input type="range" min="${start_value}" max="${end_value}" step="${increment_value}" value="${current_value}" class="slider" id="slider-input"><br>`;
    input_html += _make_buttons(button, false);

    $(content_selector).html(display_html);
    $(input_selector).html(input_html)

    var slider = document.getElementById("slider-input");
    var output = document.getElementById("slider-value-display");
    output.innerHTML = slider.value;

    slider.oninput = function() {
        output.innerHTML = this.value;
    }

    _prompt(
        parent_selector,
        input_selector,
        _get_slider_value,
        input_selector,
        callback_fn,
        seconds_before_enabling_input,
    );
}

function time_entry_prompt(
    content,
    button,
    callback_fn,
    args = [],
    seconds_before_enabling_input = 0
) {

    if (args.length > 2) {
        alert("time_entry_prompt only accepts one arg, the time to show and the minutes between intervals");
    }
    var minute_intervals;
    var displayed_time;
    if (args.length > 0) {
        minute_intervals = parseInt(args[0]);
    } else {
        minute_intervals = 5
    }
    if (args.length > 1) {
        displayed_time = args[1];
    }

    var parent_selector = "#col-1";
    var content_selector = "#col-1-content";
    var input_selector = "#col-1-input";

    var display_html = _prepare_content(content)
    var input_html = `<input type="text" class="timepicker"/> <br>`
    input_html += _make_buttons(button, false)

    $(content_selector).html(display_html)
    $(input_selector).html(input_html)

    var date;
    if (typeof displayed_time === "undefined") {
        date = new Date();
    } else {
        date = _parse_time(displayed_time)
    }
    var options = {
        now: `${date.getHours()}:${date.getMinutes() + (minute_intervals - date.getMinutes() % minute_intervals)}`,
        showSeconds: false, //Whether or not to show seconds,
        timeSeparator: ':', // The string to put in between hours and minutes (and seconds)
        minutesInterval: 5, //Change interval for minutes, defaults to 1
    };
    $('.timepicker').wickedpicker(options);

    _prompt(
        parent_selector,
        input_selector,
        _get_time_value,
        input_selector,
        callback_fn,
        seconds_before_enabling_input,
    );
}

function _parse_time(t) {
    var d = new Date();
    var time = t.match(/(\d+)(?::(\d\d))?\s*(p?)/);
    d.setHours(parseInt(time[1]) + (time[3] ? 12 : 0));
    d.setMinutes(parseInt(time[2]) || 0);
    return d;
}

function multiple_choice_prompt(
     content,
     buttons,
     callback_fn,
     args = [],
     seconds_before_enabling_input = 0
 ) {
     if (args.length > 0) {
         alert("No args accepted to multiple choice");
     }

     _two_col_prompt(
         content,
         _make_buttons(buttons),
         _get_pushed_button_value,
         undefined,
         callback_fn,
         seconds_before_enabling_input
     );
 }

function multiple_choice_prompt_one_col(
    content,
    buttons,
    callback_fn,
    args = [],
    seconds_before_enabling_input = 0
) {
    if (args.length > 0) {
        alert("No args accepted to multiple choice");
    }

    var parent_selector = "#col-1";
    var content_selector = "#col-1-content";
    var input_selector = "#col-1-input";

    $(content_selector).html(_prepare_content(content));
    $(input_selector).html(_make_buttons(buttons));

    _prompt(
        parent_selector,
        input_selector,
        _get_pushed_button_value,
        undefined,
        callback_fn,
        seconds_before_enabling_input
    );
}

function numpad_prompt(
    content,
    buttons,
    callback_fn,
    args = [],
    seconds_before_enabling_input = 0
) {
    var parent_selector = "#col-1";
    var content_selector = "#col-1-content";
    var input_selector = "#col-1-input";

    var display_html = _make_keypad_html();

    var input_html = '<input type="text" id="text-input"> <br>';
    input_html += _make_buttons(buttons, false);

    $(content_selector).html(display_html);
    $(input_selector).html(input_html);

    $(".numpad-button").each(function() {
        var value = $(this).val();
        $(this).click(function(){
            _add_value_to_text_field(value);
        })
    });

    $("#delete-button").click(_delete_character_from_field);

    _prompt(
        parent_selector,
        input_selector,
        _get_text_value,
        input_selector,
        callback_fn,
        seconds_before_enabling_input,
    );
}

function _two_col_prompt(
    content,
    input,
    get_value_fn,
    selector_to_element_of_interest,
    callback_fn,
    seconds_before_enabling_input
) {

    var parent_selector = "#col-2";
    var content_selector = "#col-2-left";
    var input_selector = "#col-2-right";

    $(content_selector).html(_prepare_content(content));
    $(input_selector).html(input);

    _prompt(
        parent_selector,
        input_selector,
        get_value_fn,
        selector_to_element_of_interest,
        callback_fn,
        seconds_before_enabling_input
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
    seconds_before_enabling_input = 0
) {

    $(input_selector).children("input:button").prop("disabled", true);
    sleep(seconds_before_enabling_input).then(function() {
        $(input_selector).children("input:button").prop("disabled", false)
    });

    _show_element_and_hide_siblings(parent_selector);

    $(input_selector).children("input:button").click(function() {

        var value = get_value_fn(this, selector_to_element_of_interest);

        // Make sure that only one button can be pushed and that it can only be pushed once
        $(input_selector).children("input:button").prop("disabled", "true");

        if (typeof callback_fn !== "undefined") {
            callback_fn(value);
        }
        $(parent_selector).fadeOut();
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

function _get_slider_value(_, parent_selector) {
    var slider_inputs = $(parent_selector).children("input.slider")
    if (slider_inputs.length != 1) {
        throw "There should only be one text entry element"
    }
    return slider_inputs[0].value
}

function _get_time_value(_, _) {
    return $('.timepicker').wickedpicker('time')
}

function _make_keypad_html() {
    return `
        <div class="numpad-row">
            <button type="button" class="numpad-button" value="1">1</button>
            <button type="button" class="numpad-button" value="2">2</button>
            <button type="button" class="numpad-button" value="3">3</button> 
            <button type="button" class="numpad-button" value="4">4</button>
        </div>
        <div class="numpad-row">
            <button type="button" class="numpad-button" value="5">5</button>
            <button type="button" class="numpad-button" value="6">6</button> 
            <button type="button" class="numpad-button" value="7">7</button>
            <button type="button" class="numpad-button" value="8">8</button>
        </div>
        <div class="numpad-row">
            <button type="button" class="numpad-button" value="9">9</button> 
            <button type="button" class="numpad-button" value="0">0</button>
            <button type="button" class="numpad-button" value="/">/</button>
            <button type="button" class="numpad-button" value=".">.</button>
            <button type="button" id="delete-button" value="&lt;">&lt;</button> 
        </div>
    `
}

function _add_value_to_text_field(button_value) {
    document.getElementById("text-input").value += button_value;
}

function _delete_character_from_field() {
    var text_field = document.getElementById("text-input");
    var current_value = text_field.value;
    text_field.value = current_value.substring(0, current_value.length-1);
}

function _is_valid_text_entry(entry) {
    return entry.length > 0
}

function _log_value(value) {
    console.log(value)
}

function sleep(seconds) {
    return new Promise(resolve => setTimeout(resolve, seconds * 1000));
}