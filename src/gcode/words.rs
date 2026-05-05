#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Word {
    pub letter: u8,
    pub value: f64,
}

pub fn strip_comments(line: &str) -> String {
    let mut out = String::new();
    let mut depth = 0i32;

    for c in line.chars() {
        match c {
            '(' => depth += 1,
            ')' if depth > 0 => depth -= 1,
            ';' if depth == 0 => break,
            _ if depth == 0 => out.push(c),
            _ => {}
        }
    }

    out.trim().to_string()
}

pub fn parse_words(line: &str) -> Vec<Word> {
    let clean = strip_comments(line);
    let bytes = clean.as_bytes();
    let mut words = Vec::new();
    let mut i = 0;

    while i < bytes.len() {
        let b = bytes[i];
        if !b.is_ascii_alphabetic() {
            i += 1;
            continue;
        }

        let letter = b.to_ascii_uppercase();
        i += 1;
        let value_start = i;
        i = number_end(bytes, i);

        if value_start == i {
            continue;
        }

        if let Ok(value) = clean[value_start..i].parse::<f64>() {
            words.push(Word { letter, value });
        }
    }

    words
}

pub fn has_word(line: &str, letter: u8) -> bool {
    let letter = letter.to_ascii_uppercase();
    parse_words(line).iter().any(|w| w.letter == letter)
}

fn number_end(bytes: &[u8], mut i: usize) -> usize {
    if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
        i += 1;
    }

    let mut saw_digit = false;
    while i < bytes.len() && bytes[i].is_ascii_digit() {
        saw_digit = true;
        i += 1;
    }

    if i < bytes.len() && bytes[i] == b'.' {
        i += 1;
        while i < bytes.len() && bytes[i].is_ascii_digit() {
            saw_digit = true;
            i += 1;
        }
    }

    if saw_digit && i < bytes.len() && (bytes[i] == b'e' || bytes[i] == b'E') {
        let exp_start = i;
        i += 1;
        if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
            i += 1;
        }
        let digits_start = i;
        while i < bytes.len() && bytes[i].is_ascii_digit() {
            i += 1;
        }
        if digits_start == i {
            i = exp_start;
        }
    }

    i
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strips_comments() {
        assert_eq!(strip_comments("G0 X1 (ignore) Y2 ; tail"), "G0 X1  Y2");
        assert_eq!(strip_comments("$$"), "$$");
    }

    #[test]
    fn parses_signed_and_exponent_words() {
        let words = parse_words("g1 x+1.5 y-2 z1e-3");
        assert_eq!(
            words[0],
            Word {
                letter: b'G',
                value: 1.0
            }
        );
        assert_eq!(
            words[1],
            Word {
                letter: b'X',
                value: 1.5
            }
        );
        assert_eq!(
            words[2],
            Word {
                letter: b'Y',
                value: -2.0
            }
        );
        assert_eq!(
            words[3],
            Word {
                letter: b'Z',
                value: 0.001
            }
        );
    }

}
